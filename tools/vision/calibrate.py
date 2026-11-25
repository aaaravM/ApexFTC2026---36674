#!/usr/bin/env python3
"""
Calibrate the Arducam OV9281 (or any USB camera) and emit numbers the FTC SDK can use.

WHY THIS MUST HAPPEN BEFORE ANY OTHER VISION WORK
    RobotConstants currently holds Logitech C270 intrinsics (fx = fy = 822.317 at 640x480), read out
    of the SDK's own built-in calibration table.  Those are now WRONG: different sensor, different
    lens, different resolution.  And fx is a multiplier on everything downstream --

        AprilTag pose:   range scales with fx
        ball distance:   d = fx * D / (2 * r_px)

    so a 20% error in fx is a 20% error in every distance the robot believes, with no warning and no
    exception.  Everything else in the vision pipeline is built on this number.

WHAT YOU NEED
    A checkerboard.  Print one, tape it FLAT to something rigid -- stiff cardboard or clipboard, not
    a bent sheet of paper, because a 1 mm bow in the board becomes a systematic error in fx.  Then
    MEASURE one square with calipers or a steel rule and pass the real number via --square-mm.
    Printers lie: "Fit to page" will silently rescale and your measured square will not match the
    nominal one.  The measured value is what matters.

    Default pattern is 9x6 INNER CORNERS.  That is the count of interior corner points, not squares
    -- a board with 10x7 squares has 9x6 inner corners.  Getting this wrong makes detection fail
    completely rather than subtly, which is at least honest.

USAGE
    python tools/vision/calibrate.py --camera 1 --width 800 --height 600 --square-mm 25
    python tools/vision/calibrate.py --camera 1 --cols 9 --rows 6 --square-mm 24.5

KEYS
    SPACE   keep this view (only offered when the full pattern is detected)
    C       run the calibration now with the views collected so far
    D       drop the most recent view
    Q/ESC   quit

HOW TO GET A GOOD CALIBRATION
    Twenty views, and make them DIFFERENT.  The single most common mistake is twenty views of the
    board held flat and square to the lens: that is nearly the same measurement twenty times, and it
    leaves focal length and distortion poorly separated, so the solver happily returns a confident
    wrong answer with a low reprojection error.

    Vary deliberately:
      - tilt the board steeply, +/-30 to 45 degrees, in both axes
      - rotate it in the image plane
      - fill the frame, then back off so it is small
      - put it in each CORNER of the frame, not just the middle.  Distortion is strongest at the
        edges, so if the board never visits the corners the distortion terms are guesswork.

    Reprojection error under about 0.5 px is good for this kind of camera, and under 1.0 px is
    usable.  Above that, something is wrong: a bent board, a wrong --square-mm, motion blur, or too
    few distinct poses.  This script warns you rather than letting a bad number through quietly.

    Calibrate at the resolution you will RUN at.  If you later change resolution, scale fx, fy, cx
    and cy by the ratio -- but note that switching between 4:3 and 16:10 on this sensor may CROP
    rather than scale, which moves the principal point and is not a simple multiply.
"""

import argparse
import os
import sys

import cv2
import numpy as np

# From Get-CimInstance Win32_PnPEntity on the team's laptop:
#   Arducam OV9281 USB Camera -> USB\VID_0C45&PID_6366
# The FTC SDK matches calibration entries by USB vendor/product id, in DECIMAL, so:
ARDUCAM_VID_HEX, ARDUCAM_PID_HEX = 0x0C45, 0x6366


def open_camera(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit("Could not open camera %d. Close anything else using it, or try another index."
                 % index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    gw = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    gh = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if (gw, gh) != (width, height):
        sys.exit(
            "Camera substituted %dx%d for the requested %dx%d.\n"
            "Calibrating at a resolution you did not ask for produces intrinsics for the WRONG\n"
            "size, which is exactly the failure this whole script exists to prevent.\n"
            "Run tools/vision/probe_camera.py to see which modes this camera really supports."
            % (gw, gh, width, height))
    return cap, gw, gh


def pose_is_new(corners, kept_corners, min_shift_px):
    """Reject a view that is nearly a duplicate of one already kept.

    Without this it is far too easy to hold the board still and bank fifteen views that carry one
    view's worth of information.  Comparing mean corner displacement is crude but it reliably
    catches 'I did not actually move it'.
    """
    c = corners.reshape(-1, 2)
    for k in kept_corners:
        if np.mean(np.linalg.norm(c - k.reshape(-1, 2), axis=1)) < min_shift_px:
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description="Calibrate a USB camera for FTC vision.")
    ap.add_argument("--camera", type=int, default=1, help="camera index (the Arducam was 1)")
    ap.add_argument("--width", type=int, default=800, help="capture width to calibrate at")
    ap.add_argument("--height", type=int, default=600, help="capture height")
    ap.add_argument("--cols", type=int, default=9, help="INNER corners across")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="MEASURED square size in mm -- measure it, do not trust the printer")
    ap.add_argument("--views", type=int, default=20, help="views to collect before calibrating")
    ap.add_argument("--min-shift", type=float, default=18.0,
                    help="mean corner shift in px required for a view to count as new")
    ap.add_argument("--outdir", default="tools/vision/calibration")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    cap, width, height = open_camera(args.camera, args.width, args.height)
    os.makedirs(args.outdir, exist_ok=True)

    # Object points: the board's corners in its own frame, in millimetres. Z = 0, it is planar.
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm

    obj_points, img_points, kept = [], [], []
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    subpix_crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    win = "calibrate  --  SPACE=keep  C=calibrate now  D=drop last  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    print()
    print("Calibrating at %dx%d, pattern %dx%d inner corners, square %.2f mm."
          % (width, height, args.cols, args.rows, args.square_mm))
    print("Collect %d DIFFERENT views: tilt steeply, rotate, near and far, and visit all four"
          % args.views)
    print("corners of the frame. Twenty flat-on views are one view repeated twenty times.")
    print()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Dropped frame; camera may have been unplugged.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)

        view = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        refined = None
        if found:
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_crit)
            cv2.drawChessboardCorners(view, pattern, refined, True)

        fresh = found and pose_is_new(refined, kept, args.min_shift)
        if found:
            msg = "PATTERN FOUND - SPACE to keep" if fresh else "too similar to a kept view - MOVE IT"
            colour = (0, 255, 0) if fresh else (0, 180, 255)
        else:
            msg = "no pattern - check --cols/--rows, lighting, and that the board is fully in frame"
            colour = (0, 0, 255)

        cv2.putText(view, "%d/%d views" % (len(obj_points), args.views), (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(view, msg, (8, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
        cv2.imshow(win, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" ") and fresh:
            obj_points.append(objp.copy())
            img_points.append(refined)
            kept.append(refined.copy())
            cv2.imwrite(os.path.join(args.outdir, "view_%02d.png" % len(obj_points)), frame)
            print("  kept view %d/%d" % (len(obj_points), args.views))
        elif key == ord("d") and obj_points:
            obj_points.pop(); img_points.pop(); kept.pop()
            print("  dropped; %d left" % len(obj_points))
        elif key == ord("c"):
            break

        if len(obj_points) >= args.views:
            print("\nGot %d views. Calibrating..." % len(obj_points))
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < 6:
        sys.exit("\nOnly %d views -- too few to calibrate. Aim for %d."
                 % (len(obj_points), args.views))

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (width, height), None, None)

    # Per-view reprojection error: a single bad view (board moved during capture, or a mis-detected
    # corner) can drag the whole fit, and the overall RMS hides it. Show them individually.
    per_view = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        per_view.append(err)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k = dist.ravel()
    hfov = 2 * np.degrees(np.arctan(width / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(height / (2 * fy)))

    print()
    print("=" * 78)
    print("CALIBRATION RESULT   %dx%d, %d views" % (width, height, len(obj_points)))
    print("=" * 78)
    print("  fx = %.3f    fy = %.3f" % (fx, fy))
    print("  cx = %.3f    cy = %.3f" % (cx, cy))
    print("  distortion (k1 k2 p1 p2 k3) = %s" % np.array2string(k[:5], precision=6))
    print("  RMS reprojection error = %.4f px" % rms)
    print("  worst single view      = %.4f px" % max(per_view))
    print("  implied FOV            = %.1f deg horizontal, %.1f deg vertical" % (hfov, vfov))
    print()

    # ---- Sanity checks. A low RMS on a bad set of views is the trap, so check the physics too. ----
    problems = []
    if rms > 1.0:
        problems.append("RMS %.2f px is too high. Likely a bent board, a wrong --square-mm, motion "
                        "blur, or too few distinct poses." % rms)
    elif rms > 0.5:
        problems.append("RMS %.2f px is usable but not great; more varied poses would improve it."
                        % rms)
    if abs(fx - fy) / max(fx, fy) > 0.05:
        problems.append("fx and fy differ by %.1f%%. Square pixels should give near-equal values; "
                        "this suggests a bad fit or a non-square capture mode."
                        % (100 * abs(fx - fy) / max(fx, fy)))
    if abs(cx - width / 2.0) > width * 0.10 or abs(cy - height / 2.0) > height * 0.10:
        problems.append("Principal point (%.0f, %.0f) is more than 10%% off centre (%.0f, %.0f). "
                        "Possible, but usually means the board never reached the frame corners."
                        % (cx, cy, width / 2.0, height / 2.0))
    if max(per_view) > 3 * (sum(per_view) / len(per_view)):
        worst = per_view.index(max(per_view)) + 1
        problems.append("View %d has %.1fx the average error. Re-run and drop it (its image is "
                        "saved as view_%02d.png)." % (worst, max(per_view) / (sum(per_view)/len(per_view)), worst))

    if problems:
        print("CHECK THESE:")
        for p in problems:
            print("  !  %s" % p)
    else:
        print("All sanity checks passed.")
    print()
    print("Compare that FOV against Arducam's spec for the M12 lens. If the spec says, for example,")
    print("70 deg and you measured %.0f deg, something is wrong with the calibration -- not with" % hfov)
    print("the lens. That cross-check catches errors a low reprojection error will not.")
    print()

    # ---- Output 1: Java, for RobotConstants ----
    java = """    // ---- Lens intrinsics: Arducam B0332 (OV9281 mono global shutter) + M12 lens ----
    //
    // MEASURED on %d views at %dx%d with a %.2f mm checkerboard.
    // RMS reprojection error %.4f px, worst view %.4f px.
    // Implied FOV: %.1f deg horizontal, %.1f deg vertical.
    //
    // These REPLACE the Logitech C270 values (fx = 822.317), which belonged to a different
    // camera and a different lens and are wrong for this hardware.

    /** Resolution these intrinsics were measured at. Scale with fxAt()/cxAt() for any other. */
    public static final int CAL_WIDTH  = %d;
    public static final int CAL_HEIGHT = %d;

    public static final double FX = %.3f;
    public static final double FY = %.3f;
    public static final double CX = %.3f;
    public static final double CY = %.3f;

    /** Distortion k1, k2, p1, p2, k3. The SDK's AprilTag path takes only the four above. */
    public static final double[] DISTORTION = { %.6f, %.6f, %.6f, %.6f, %.6f };
""" % (len(obj_points), width, height, args.square_mm, rms, max(per_view), hfov, vfov,
       width, height, fx, fy, cx, cy, k[0], k[1], k[2], k[3], k[4])

    java_path = os.path.join(args.outdir, "RobotConstants_Camera_snippet.java")
    with open(java_path, "w") as f:
        f.write(java)

    # ---- Output 2: teamwebcamcalibrations.xml, so the SDK applies it automatically ----
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Arducam B0332 (OV9281 mono global shutter), measured %d views at %dx%d,
  RMS reprojection error %.4f px.

  The SDK matches a camera by USB vendor/product id, in DECIMAL. From Windows this device
  reports USB\\VID_0C45&PID_6366, so vid="%d" pid="%d".

  VERIFY the install path and that the SDK picks this up before trusting it: the calibration
  silently falling back to a default is exactly the failure mode that wastes a weekend. Confirm
  by checking the focal length the SDK reports at init against %.3f.
-->
<Calibrations>
    <Camera name="Arducam OV9281" vid="%d" pid="%d">
        <Calibration size="%d %d"
                     focalLength="%.3ff, %.3ff"
                     principalPoint="%.3ff, %.3ff"
                     distortionCoefficients="%.6f, %.6f, %.6f, %.6f, %.6f, 0, 0, 0"/>
    </Camera>
</Calibrations>
""" % (len(obj_points), width, height, rms,
       ARDUCAM_VID_HEX, ARDUCAM_PID_HEX, fx,
       ARDUCAM_VID_HEX, ARDUCAM_PID_HEX,
       width, height, fx, fy, cx, cy, k[0], k[1], k[2], k[3], k[4])

    xml_path = os.path.join(args.outdir, "teamwebcamcalibrations.xml")
    with open(xml_path, "w") as f:
        f.write(xml)

    np.savez(os.path.join(args.outdir, "calibration.npz"), K=K, dist=dist,
             width=width, height=height, rms=rms, square_mm=args.square_mm)

    print("WROTE")
    print("  %s" % java_path)
    print("  %s" % xml_path)
    print("  %s" % os.path.join(args.outdir, "calibration.npz"))
    print()
    print(java)


if __name__ == "__main__":
    main()
