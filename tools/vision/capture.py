#!/usr/bin/env python3
"""
Dataset capture for the BIOBUZZ ball detector.

WHAT THIS IS FOR
    Whatever the final detector turns out to be, it needs footage of the real balls:

      - classical OpenCV       -> you need frames to tune and then REGRESSION-TEST thresholds
                                  against, so a lighting change does not silently break you
      - a trained model        -> frames ARE the dataset; this is the thing you label
      - a Limelight pipeline   -> same, its colour and neural pipelines are tuned on real images

    So this footage is never wasted work, which is why it is worth collecting before the
    architecture is settled.

THE MISTAKE THIS SCRIPT EXISTS TO PREVENT
    Recording 30 frames per second of a ball sitting still gives you 900 nearly identical images.
    A model trained on that learns one pose under one light and fails at a competition.  What you
    actually want is FEW frames but DIVERSE ones.  So this script samples on an interval by default
    (2 per second) and prints a shot list to work through.

    Diversity that matters here, roughly in order:
      1. Distance -- close, mid, far, and right at the edge of usable range
      2. Lighting -- bright, dim, side-lit, backlit, under the actual venue lights if you can
      3. Occlusion -- balls touching, overlapping, half behind a robot or a field wall
      4. Position in frame -- centre, corners, partly cut off at the edge
      5. Motion blur -- rolling balls, and the camera panning
      6. Hard negatives -- the field WITHOUT balls, especially anything of a similar colour:
         alliance-coloured field elements, tape, another robot's bumper, a red or blue shirt
      7. Backgrounds -- over the field tiles, over the field wall, with people behind

    Item 6 is the one teams skip and then wonder why the robot chases a referee's shirt.  Shoot
    plenty of empty-field frames.  A detector learns what a ball is NOT from those.

USAGE
    python tools/vision/capture.py --label pollen
    python tools/vision/capture.py --label empty_field --interval 0.5
    python tools/vision/capture.py --label nectar_red --video       # record video instead of stills
    python tools/vision/capture.py --label mixed --width 640 --height 480

KEYS
    SPACE   capture one frame right now (manual mode, regardless of interval)
    A       toggle automatic interval capture on/off
    V       start / stop video recording
    Q, ESC  quit

A NOTE ON RESOLUTION
    Default is 640x480.  That is deliberate and differs from the tuner's default of 320x240:

      - the TUNER matches the robot's live capture resolution, because the pixel-radius filter it
        produces is resolution-specific
      - CAPTURE wants the higher resolution, because you can always downscale a 640 image to 320
        later, and you cannot go the other way.  Capture high, train or test at whatever you like.

    Stay at 4:3.  The C270's built-in SDK calibration only auto-applies at 4:3; any 16:9 mode makes
    the SDK silently fall back to a generic focal length that is about 42% wrong.  Do not collect a
    dataset in a mode the robot cannot use.
"""

import argparse
import os
import sys
import time

import cv2

SHOT_LIST = """
    SHOT LIST -- aim for roughly this spread, about 150-300 frames per ball type total

      [ ]  20 frames   one ball, 12-24 in from the lens, moved around the frame
      [ ]  20 frames   one ball, 36-60 in
      [ ]  20 frames   one ball, 72-120 in, and note where it becomes too small to see
      [ ]  20 frames   one ball at the very edge of frame, partly cut off
      [ ]  20 frames   two or more balls touching and overlapping
      [ ]  20 frames   balls rolling -- expect motion blur, that is the point
      [ ]  15 frames   dim light, and lights off on one side
      [ ]  15 frames   backlit, camera pointed toward a window or a light
      [ ]  30 frames   HARD NEGATIVES: no balls at all, but anything similarly coloured in shot --
                       field tape, alliance-coloured structures, a red or blue shirt, another robot
      [ ]  20 frames   from the camera's REAL mounted height and angle on the robot, not desk height

    The last one is the most important and the easiest to skip.  A detector trained only on
    desk-height footage sees a different world than the robot does.
"""


def open_camera(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit(
            "Could not open camera index %d.\n"
            "  - Close anything else using it (Zoom, Teams, OBS, the Camera app).\n"
            "  - Try --camera 0, 1, 2 ...\n"
            "  - A charge-only USB cable looks exactly like this failure." % index
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    got_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    got_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if (got_w, got_h) != (width, height):
        print("\nWARNING: asked for %dx%d, camera gave %dx%d. Saving at %dx%d."
              % (width, height, got_w, got_h, got_w, got_h))
        print("Check the aspect ratio is still 4:3 before you collect a whole dataset.\n")
    return cap, got_w, got_h


def main():
    ap = argparse.ArgumentParser(description="Capture ball footage for tuning or training.")
    ap.add_argument("--label", required=True,
                    help="what is in shot: pollen, nectar_red, nectar_blue, mixed, empty_field, ...")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--interval", type=float, default=0.5,
                    help="seconds between automatic captures; 0.5 = 2 per second")
    ap.add_argument("--video", action="store_true", help="start in video-recording mode")
    ap.add_argument("--outdir", default="tools/vision/dataset")
    args = ap.parse_args()

    cap, width, height = open_camera(args.camera, args.width, args.height)

    out_frames = os.path.join(args.outdir, args.label)
    os.makedirs(out_frames, exist_ok=True)

    # Continue the numbering rather than overwriting, so several sessions accumulate.
    existing = [f for f in os.listdir(out_frames) if f.endswith(".png")]
    n = len(existing)
    if n:
        print("Found %d existing frames in %s -- continuing from %d." % (n, out_frames, n))

    writer = None
    auto = False
    last_auto = 0.0
    session_start = time.monotonic()

    print(SHOT_LIST)
    print("Capturing %dx%d, label '%s' -> %s" % (width, height, args.label, out_frames))
    print("SPACE = one frame   A = auto on/off   V = video   Q = quit")
    print()

    if args.video:
        print("Press V to begin recording.")

    win = "capture  --  SPACE=frame  A=auto  V=video  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Dropped frame; camera may have been unplugged.")
            break

        now = time.monotonic()
        took = None

        if auto and (now - last_auto) >= args.interval:
            took = frame
            last_auto = now

        if writer is not None:
            writer.write(frame)

        preview = frame.copy()
        status = "%s  n=%d  %s%s" % (
            args.label, n,
            "AUTO %.1fs" % args.interval if auto else "manual",
            "  REC" if writer is not None else "",
        )
        cv2.putText(preview, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(preview, "%.0fs elapsed" % (now - session_start), (8, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        if writer is not None:
            cv2.circle(preview, (width - 18, 18), 7, (0, 0, 255), -1)
        cv2.imshow(win, preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            took = frame
        elif key == ord("a"):
            auto = not auto
            last_auto = now
            print("auto capture %s" % ("ON" if auto else "OFF"))
        elif key == ord("v"):
            if writer is None:
                path = os.path.join(args.outdir, "%s_%dx%d_%03d.avi" % (args.label, width, height, n))
                # MJPG in an AVI container: universally readable, and no codec to install on Windows.
                # The file is large, which does not matter for a few minutes of footage.
                writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 30.0, (width, height))
                if not writer.isOpened():
                    print("Could not open a video writer; staying in stills mode.")
                    writer = None
                else:
                    print("recording -> %s" % path)
            else:
                writer.release()
                writer = None
                print("recording stopped")

        if took is not None:
            path = os.path.join(out_frames, "%s_%05d.png" % (args.label, n))
            cv2.imwrite(path, took)
            n += 1
            if n % 10 == 0:
                print("  %d frames" % n)

    if writer is not None:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()

    print()
    print("%d frames total in %s" % (n, out_frames))
    print()
    print("These are NOT committed to git -- a few hundred PNGs would bloat the repo and nobody")
    print("needs them on the robot. Put them in shared Drive storage so the whole team can label")
    print("them. tools/vision/dataset/ is in .gitignore for this reason.")


if __name__ == "__main__":
    main()
