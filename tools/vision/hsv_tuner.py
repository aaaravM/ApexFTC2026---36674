#!/usr/bin/env python3
"""
HSV threshold tuner for the BIOBUZZ ball detector.

WHAT THIS IS FOR
    The robot's ball detector works by asking "which pixels are this colour?".  That question needs
    six numbers -- a low and a high bound for Hue, Saturation and Value.  Guessing them does not
    work.  This tool lets you point the real camera at the real balls under the real lights, drag
    six sliders until only the balls light up, and then press P to print the numbers in a form you
    can paste straight into BallTracker.java.

    Those six numbers are the single highest-value measurement you can take right now.  They are
    needed whether the final detector is classical OpenCV, a trained model (you still need them to
    sanity-check your labels), or a Limelight colour pipeline (its UI wants the same six numbers).

WHY RESOLUTION MATTERS, AND WHY THIS SCRIPT NAGS ABOUT IT
    Focal length scales linearly with capture width.  The SDK's built-in C270 calibration is
    fx = 822.317 at 640x480.  At 320x240 the correct value is 411.159.  Using the 640 number on a
    320 image doubles every distance the robot reports, and nothing errors -- it just quietly aims
    wrong.  So this script captures at the SAME resolution the robot uses, scales fx to match, and
    refuses to be quiet if the webcam hands back a different resolution than you asked for.

    Windows is the reason for that last check.  A USB camera that cannot do the mode you requested
    will often return the nearest mode it CAN do, with no error at all.

USAGE
    python tools/vision/hsv_tuner.py
    python tools/vision/hsv_tuner.py --width 640 --height 480
    python tools/vision/hsv_tuner.py --camera 1          # if the laptop's built-in cam is index 0
    python tools/vision/hsv_tuner.py --preset nectar_red

KEYS
    P   print the current six numbers as pasteable Java
    S   save the current frame and mask as PNGs (builds a dataset while you tune)
    1   load the POLLEN preset          2  NECTAR_RED       3  NECTAR_BLUE
    M   cycle view: overlay -> mask -> side by side
    [   ]   decrease / increase the minimum blob radius
    Q or ESC   quit

READING THE SLIDERS
    Hue is the colour itself, 0-179 in OpenCV (NOT 0-359 -- OpenCV halves it to fit in a byte).
    Saturation is how vivid, 0-255; low saturation is washed out or grey.
    Value is how bright, 0-255.

    Tune in this order and it goes much faster:
      1. Widen everything, then narrow HUE until the ball is still fully lit but the floor is not.
      2. Raise SAT_LOW until grey and white things (field tape, the tiles, a shiny reflection) drop
         out.  This does most of the work.
      3. Raise VAL_LOW until dark shadows drop out.  Keep VAL_HIGH at 255 unless a specular
         highlight on the ball is blowing out to white.
      4. Only then narrow SAT_HIGH / VAL_HIGH, and usually you will not need to.

    RED IS A SPECIAL CASE.  Red sits at both ends of the hue circle -- roughly 0-10 and 170-179 --
    so a single low-to-high range cannot capture it.  Press 2 for the NECTAR_RED preset and this
    tool switches to WRAPPED mode automatically: it builds two masks and ORs them.  The printed
    Java reflects that.  If you tune red with a single range you will lose half the ball.
"""

import argparse
import os
import sys

import cv2
import numpy as np

# ----------------------------------------------------------------------------------------------
# These mirror RobotConstants.Camera and BallType.  Keep them in sync if the Java changes.
# ----------------------------------------------------------------------------------------------

# Logitech C270, read from the SDK's own table (RobotCore-12.0.0.aar, builtinwebcamcalibrations.xml)
CAL_WIDTH = 640
CAL_FX = 822.317

BALL_DIAMETER_IN = {
    "pollen": 2.8,
    "nectar_red": 3.6,
    "nectar_blue": 3.6,
}

# Starting points, NOT measured values.  Replace them with what you measure.
PRESETS = {
    # name:        (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped)
    "pollen": (20, 90, 90, 38, 255, 255, False),
    "nectar_red": (0, 110, 80, 10, 255, 255, True),
    "nectar_blue": (95, 110, 70, 125, 255, 255, False),
}

WINDOW = "HSV tuner  --  P=print  S=save  1/2/3=preset  M=view  []=radius  Q=quit"


def open_camera(index, width, height):
    """Open the camera and force a resolution, loudly, because Windows will not tell you."""
    # CAP_DSHOW is the DirectShow backend.  On Windows it opens far faster than the default MSMF
    # backend and actually honours resolution requests, which MSMF sometimes ignores.
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)  # fall back, in case DSHOW is unavailable
    if not cap.isOpened():
        sys.exit(
            "Could not open camera index %d.\n"
            "  - Is anything else using it?  Close Zoom, Teams, OBS, the Camera app.\n"
            "  - Try another index: --camera 0, 1, 2 ...\n"
            "  - A charge-only USB cable will also look exactly like this." % index
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    got_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    got_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if (got_w, got_h) != (width, height):
        print()
        print("!" * 78)
        print("WARNING: asked for %dx%d, camera gave %dx%d." % (width, height, got_w, got_h))
        print("The HSV numbers will still be valid -- colour does not depend on resolution.")
        print("But any DISTANCE shown below is computed for %d wide, so treat it as indicative" % got_w)
        print("only, and make sure the robot requests a resolution the camera can actually do.")
        print("!" * 78)
        print()
    return cap, got_w, got_h


def nothing(_):
    pass


def build_ui(initial):
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 900, 700)
    h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, _ = initial
    cv2.createTrackbar("H low", WINDOW, h_lo, 179, nothing)
    cv2.createTrackbar("H high", WINDOW, h_hi, 179, nothing)
    cv2.createTrackbar("S low", WINDOW, s_lo, 255, nothing)
    cv2.createTrackbar("S high", WINDOW, s_hi, 255, nothing)
    cv2.createTrackbar("V low", WINDOW, v_lo, 255, nothing)
    cv2.createTrackbar("V high", WINDOW, v_hi, 255, nothing)
    # Wrapped mode is for red, which straddles hue 0.  1 = two ranges ORed together.
    cv2.createTrackbar("wrapped", WINDOW, 1 if initial[6] else 0, 1, nothing)


def read_ui():
    g = lambda n: cv2.getTrackbarPos(n, WINDOW)
    return (
        g("H low"), g("S low"), g("V low"),
        g("H high"), g("S high"), g("V high"),
        g("wrapped") == 1,
    )


def apply_preset(name):
    p = PRESETS[name]
    cv2.setTrackbarPos("H low", WINDOW, p[0])
    cv2.setTrackbarPos("S low", WINDOW, p[1])
    cv2.setTrackbarPos("V low", WINDOW, p[2])
    cv2.setTrackbarPos("H high", WINDOW, p[3])
    cv2.setTrackbarPos("S high", WINDOW, p[4])
    cv2.setTrackbarPos("V high", WINDOW, p[5])
    cv2.setTrackbarPos("wrapped", WINDOW, 1 if p[6] else 0)


def make_mask(hsv, h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped):
    """Threshold, then clean up.

    The morphology matters more than people expect.  OPEN (erode then dilate) deletes salt-and-
    pepper speckle without shrinking real blobs.  CLOSE (dilate then erode) fills the pinholes a
    specular highlight punches in the middle of a ball, which otherwise splits one ball into a
    ring of fragments and ruins the circularity test.
    """
    if wrapped:
        # Red wraps around hue 0.  Build [0, h_lo] and [h_hi, 179] and OR them.  Note the roles of
        # h_lo and h_hi invert here: h_lo is the top of the low band, h_hi the bottom of the high.
        lo_band = cv2.inRange(hsv, np.array([0, s_lo, v_lo]), np.array([h_lo, s_hi, v_hi]))
        hi_band = cv2.inRange(hsv, np.array([h_hi, s_lo, v_lo]), np.array([179, s_hi, v_hi]))
        mask = cv2.bitwise_or(lo_band, hi_band)
    else:
        mask = cv2.inRange(hsv, np.array([h_lo, s_lo, v_lo]), np.array([h_hi, s_hi, v_hi]))

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask


def find_balls(mask, min_radius, min_circularity=0.60):
    """Contours -> circles, keeping only the round ones.

    The circularity test is what separates a ball from a stripe of field tape of the same colour.
    contourArea / (pi * r^2) is 1.0 for a perfect disc.  A real ball lands around 0.75-0.92; a
    long thin blob lands near 0.2.  0.60 is a deliberately generous floor so a partly occluded
    ball still passes -- tighten it if tape is getting through.
    """
    found = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        (x, y), r = cv2.minEnclosingCircle(c)
        if r < min_radius:
            continue
        area = cv2.contourArea(c)
        circ = area / (np.pi * r * r) if r > 0 else 0.0
        if circ < min_circularity:
            continue
        found.append((x, y, r, circ))
    found.sort(key=lambda b: -b[2])  # biggest (nearest) first
    return found


def main():
    ap = argparse.ArgumentParser(description="Tune HSV thresholds for the BIOBUZZ ball detector.")
    ap.add_argument("--camera", type=int, default=0, help="camera index; built-in webcam is usually 0")
    ap.add_argument("--width", type=int, default=320,
                    help="capture width; default 320 to match BallTracker.CAPTURE_WIDTH")
    ap.add_argument("--height", type=int, default=240, help="capture height; default 240")
    ap.add_argument("--preset", default="pollen", choices=sorted(PRESETS), help="starting preset")
    ap.add_argument("--outdir", default="tools/vision/captures", help="where S saves frames")
    args = ap.parse_args()

    cap, width, height = open_camera(args.camera, args.width, args.height)
    os.makedirs(args.outdir, exist_ok=True)

    # Scale the calibrated focal length to the resolution we are actually capturing at.
    fx = CAL_FX * width / float(CAL_WIDTH)

    build_ui(PRESETS[args.preset])
    ball_name = args.preset
    min_radius = 6
    view_mode = 0
    saved = 0

    print()
    print("Capturing %dx%d   fx = %.3f  (calibrated %.3f at %d, scaled)" % (width, height, fx, CAL_FX, CAL_WIDTH))
    print("Ball diameter assumed %.2f in for '%s'." % (BALL_DIAMETER_IN[ball_name], ball_name))
    print("Distances below use  d = fx * D / (2 * r_px)  -- the apparent-size model.")
    print()
    print("Put a ball about 36 in from the lens and check the reported distance. If it reads")
    print("consistently high or low by the same factor, the focal length or the assumed ball")
    print("diameter is wrong -- not the thresholds.")
    print()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Dropped frame; camera may have been unplugged.")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped = read_ui()
        mask = make_mask(hsv, h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped)
        balls = find_balls(mask, min_radius)

        overlay = frame.copy()
        for i, (x, y, r, circ) in enumerate(balls):
            dist = fx * BALL_DIAMETER_IN[ball_name] / (2.0 * r)
            colour = (0, 255, 0) if i == 0 else (0, 180, 255)
            cv2.circle(overlay, (int(x), int(y)), int(r), colour, 2)
            cv2.circle(overlay, (int(x), int(y)), 2, (0, 0, 255), -1)
            cv2.putText(overlay, "%.0fin r=%.0f c=%.2f" % (dist, r, circ),
                        (int(x) - int(r), int(y) - int(r) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1)

        hud = [
            "%s%s  blobs=%d  minR=%d" % (ball_name, "  WRAPPED" if wrapped else "", len(balls), min_radius),
            "H %d-%d  S %d-%d  V %d-%d" % (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi),
        ]
        for j, line in enumerate(hud):
            cv2.putText(overlay, line, (6, 14 + j * 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if view_mode == 0:
            shown = overlay
        elif view_mode == 1:
            shown = mask_bgr
        else:
            shown = np.hstack([overlay, mask_bgr])
        cv2.imshow(WINDOW, shown)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("p"):
            print_java(ball_name, h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped, min_radius)
        elif key == ord("s"):
            stem = os.path.join(args.outdir, "%s_%03d" % (ball_name, saved))
            cv2.imwrite(stem + "_frame.png", frame)
            cv2.imwrite(stem + "_mask.png", mask)
            saved += 1
            print("saved %s_frame.png and _mask.png" % stem)
        elif key == ord("m"):
            view_mode = (view_mode + 1) % 3
        elif key == ord("1"):
            ball_name = "pollen"; apply_preset(ball_name); print("preset: pollen")
        elif key == ord("2"):
            ball_name = "nectar_red"; apply_preset(ball_name); print("preset: nectar_red (wrapped)")
        elif key == ord("3"):
            ball_name = "nectar_blue"; apply_preset(ball_name); print("preset: nectar_blue")
        elif key == ord("["):
            min_radius = max(2, min_radius - 1)
        elif key == ord("]"):
            min_radius = min(100, min_radius + 1)

    cap.release()
    cv2.destroyAllWindows()
    if saved:
        print("\n%d frame/mask pairs in %s -- that is the start of a dataset, keep them." % (saved, args.outdir))


def print_java(name, h_lo, s_lo, v_lo, h_hi, s_hi, v_hi, wrapped, min_radius):
    const = name.upper()
    print()
    print("=" * 78)
    print("Measured %s thresholds -- paste into BallTracker.java" % const)
    print("Capture resolution MUST match what you tuned at, or the radius filter will be wrong.")
    print("=" * 78)
    if wrapped:
        print("""
    // %s straddles hue 0, so it needs TWO ranges ORed together. One range loses half the ball.
    private static final Scalar %s_LO_A = new Scalar(  0, %3d, %3d);
    private static final Scalar %s_HI_A = new Scalar(%3d, %3d, %3d);
    private static final Scalar %s_LO_B = new Scalar(%3d, %3d, %3d);
    private static final Scalar %s_HI_B = new Scalar(179, %3d, %3d);
    private static final int    %s_MIN_RADIUS_PX = %d;""" % (
            const,
            const, s_lo, v_lo,
            const, h_lo, s_hi, v_hi,
            const, h_hi, s_lo, v_lo,
            const, s_hi, v_hi,
            const, min_radius))
    else:
        print("""
    private static final Scalar %s_LO = new Scalar(%3d, %3d, %3d);
    private static final Scalar %s_HI = new Scalar(%3d, %3d, %3d);
    private static final int    %s_MIN_RADIUS_PX = %d;""" % (
            const, h_lo, s_lo, v_lo,
            const, h_hi, s_hi, v_hi,
            const, min_radius))
    print()
    print("Tuned at this lighting only. Re-check under the competition lights -- venue lighting is")
    print("the single most common reason a colour detector that worked at home fails at a meet.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
