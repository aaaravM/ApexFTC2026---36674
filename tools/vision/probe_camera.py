#!/usr/bin/env python3
"""
Probe a USB camera: which index it is, what modes it really supports, and whether it is monochrome.

WHY THIS EXISTS
    A USB camera will happily accept a resolution request it cannot honour and hand back the nearest
    mode it can do, with no error.  That silent substitution is the single most common way a vision
    pipeline ends up with the wrong focal length, because focal length scales with capture width.
    So before anything else: find out what this camera ACTUALLY does.

    It also answers the question you cannot answer by looking at the camera: does it deliver one
    channel or three, and if three, are they identical (i.e. is it really monochrome)?

USAGE
    python tools/vision/probe_camera.py              # probe indices 0-4
    python tools/vision/probe_camera.py --camera 1   # probe just one
    python tools/vision/probe_camera.py --fps        # also measure real framerate (slower)
"""

import argparse
import time

import cv2
import numpy as np

# Modes worth asking for. 4:3 and the OV9281's native 16:10 are both here deliberately; the report
# tells you which ones survive.
CANDIDATE_MODES = [
    (1280, 800),   # OV9281 native, 16:10
    (1280, 720),   # the "720P" on the box
    (1280, 960),
    (960, 600),
    (800, 600),    # 4:3
    (640, 480),    # 4:3, the usual AprilTag default
    (640, 400),    # 16:10, half native -- the likely 100 fps mode
    (480, 360),
    (320, 240),    # 4:3
    (320, 200),
]

FOURCCS = ["(default)", "MJPG", "YUY2", "GREY", "Y8  ", "Y800"]


def fourcc_to_str(v):
    v = int(v)
    if v == 0:
        return "0"
    try:
        return "".join([chr((v >> (8 * i)) & 0xFF) for i in range(4)])
    except ValueError:
        return hex(v)


def describe_frame(frame):
    """Say what we actually got, especially whether it is really mono."""
    if frame is None:
        return "no frame"
    shape = frame.shape
    if len(shape) == 2:
        return "%dx%d, 1 channel (true single-channel mono)" % (shape[1], shape[0])
    h, w, c = shape
    if c == 3:
        # A mono sensor delivered as BGR has three identical channels. Check rather than assume.
        b, g, r = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
        same = np.array_equal(b, g) and np.array_equal(g, r)
        if same:
            return "%dx%d, 3 channels but ALL IDENTICAL -> monochrome delivered as BGR" % (w, h)
        # Quantify how much colour there actually is.
        spread = float(np.mean(np.abs(b.astype(np.int16) - r.astype(np.int16))))
        return "%dx%d, 3 channels, genuinely different (mean |B-R| = %.1f) -> colour" % (w, h, spread)
    return "%dx%d, %d channels" % (w, h, c)


def probe_index(index, measure_fps):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return False

    print("=" * 78)
    print("CAMERA INDEX %d" % index)
    print("=" * 78)

    ok, frame = cap.read()
    if not ok:
        print("  opens but delivers no frames -- likely in use by another program")
        cap.release()
        return True

    native_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    native_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("  default mode : %dx%d" % (native_w, native_h))
    print("  fourcc       : %s" % fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC)))
    print("  reported fps : %s" % cap.get(cv2.CAP_PROP_FPS))
    print("  frame        : %s" % describe_frame(frame))
    print()

    print("  Resolution modes -- 'asked' vs what the camera actually gave:")
    print("  %-12s %-12s %s" % ("asked", "got", "verdict"))
    accepted = []
    for (w, h) in CANDIDATE_MODES:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        ok, f = cap.read()
        if not ok:
            print("  %-12s %-12s no frame" % ("%dx%d" % (w, h), "-"))
            continue
        gw = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        gh = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        exact = (gw, gh) == (w, h)
        if exact:
            accepted.append((w, h))
        print("  %-12s %-12s %s" % ("%dx%d" % (w, h), "%dx%d" % (gw, gh),
                                    "OK" if exact else "SUBSTITUTED - do not use"))
    print()

    if measure_fps and accepted:
        print("  Measured framerate (30 frames per mode, so it is rough):")
        for (w, h) in accepted:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            for _ in range(5):
                cap.read()  # let the mode settle
            t0 = time.monotonic()
            n = 0
            for _ in range(30):
                if cap.read()[0]:
                    n += 1
            dt = time.monotonic() - t0
            fps = n / dt if dt > 0 else 0
            mb = w * h * fps / 1e6
            print("    %-12s %6.1f fps   (~%.1f MB/s raw 8-bit mono)" % ("%dx%d" % (w, h), fps, mb))
        print()

    print("  Pixel-format requests (many cameras ignore these):")
    for name in FOURCCS:
        if name == "(default)":
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*name))
        ok, _ = cap.read()
        got = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
        print("    asked %-6s -> got %-6s %s" % (name.strip(), got, "" if ok else "(no frame)"))
    print()

    cap.release()
    return True


def main():
    ap = argparse.ArgumentParser(description="Probe USB cameras for real capabilities.")
    ap.add_argument("--camera", type=int, default=None, help="probe only this index")
    ap.add_argument("--max-index", type=int, default=4, help="highest index to try")
    ap.add_argument("--fps", action="store_true", help="also measure real framerate per mode")
    args = ap.parse_args()

    print()
    print("Close any other program using a camera first (Zoom, Teams, OBS, the Camera app),")
    print("or that camera will show as unavailable here.")
    print()

    indices = [args.camera] if args.camera is not None else range(args.max_index + 1)
    found = 0
    for i in indices:
        if probe_index(i, args.fps):
            found += 1

    print("=" * 78)
    print("%d camera(s) responded." % found)
    print()
    print("WHAT TO DO WITH THIS")
    print("  - Use only modes marked OK. A SUBSTITUTED mode means the camera silently gave you")
    print("    something else, and any focal length you derive for the asked-for size will be wrong.")
    print("  - Calibrate at, and run the robot at, the SAME resolution. If you must change it,")
    print("    scale fx, fy, cx, cy by the width/height ratio.")
    print("  - Note the aspect ratio. Switching between 4:3 and 16:10 is not a simple scale if the")
    print("    sensor crops rather than scales -- it moves the principal point too.")


if __name__ == "__main__":
    main()
