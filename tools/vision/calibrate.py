#!/usr/bin/env python3
"""
Calibrate the Arducam B0332 (OV9281 mono global shutter) and emit FTC-ready intrinsics.

================================================================================================
WHY THIS MUST HAPPEN BEFORE ANY OTHER VISION WORK
================================================================================================
RobotConstants currently holds LOGITECH C270 intrinsics (fx = 822.317) for a camera the team does
not own.  fx is a multiplier on everything downstream:

    AprilTag pose:  range scales linearly with fx
    ball distance:  d = fx * D / (2r), and the ground-plane model is linear in fy

So a 20% error in fx is a 20% error in every distance the robot believes, with no exception and no
symptom.  Worse, nothing in TeamCode calls AprilTagProcessor.setLensIntrinsics, which means the
AprilTag pipeline is currently falling back to the SDK's hardcoded default of fx = fy = 578.272 --
a Logitech C920 entry.

================================================================================================
CHARUCO, NOT A PLAIN CHECKERBOARD
================================================================================================
A ChArUco board is a checkerboard with ArUco markers in the white squares, so every corner is
individually IDENTIFIED rather than merely located.  Three things follow, and they all matter:

  - PARTIAL VIEWS WORK.  A plain checkerboard must be fully visible or findChessboardCorners fails
    outright.  That makes it painful to get corners into the frame CORNERS, which is exactly where
    distortion lives and exactly what a calibration needs.  With ChArUco you can hang the board
    half out of frame and still use it.
  - NO AMBIGUITY.  A symmetric checkerboard can be matched 180 degrees rotated; identified markers
    cannot.
  - PER-CORNER REJECTION.  One bad corner is dropped rather than poisoning the view.

API WARNING: cv2.aruco.interpolateCornersCharuco and cv2.aruco.calibrateCameraCharuco were REMOVED
in OpenCV 4.7+.  They do not exist in the 4.12 installed here.  Most tutorials online still use
them.  This script uses the modern cv2.aruco.CharucoDetector, and --selftest proves that path works
on this build by recovering known intrinsics from synthetic views (it recovers fx to about 0.1%).

================================================================================================
USAGE
================================================================================================
    python tools/vision/calibrate.py --selftest                 # verify the method, no camera
    python tools/vision/calibrate.py --make-board               # writes charuco_board.png -> PRINT
    python tools/vision/calibrate.py --capture --camera 1       # collect views
    python tools/vision/calibrate.py --solve                    # fit and write the outputs

PRINTING THE BOARD
    Print at 100% scale -- "fit to page" silently rescales it.  Then MEASURE a square with calipers
    and pass --square-mm with what you measured.  Tape it to GLASS or a clipboard: a 1 mm bow in the
    board becomes a systematic error in fx, and a bowed board is the single most common cause of a
    confident wrong answer.

COLLECTING VIEWS -- the part everyone gets wrong
    Twenty views of the board held flat and square to the lens is one measurement repeated twenty
    times.  It leaves focal length and distortion poorly separated, and the solver returns a
    confident wrong answer WITH A LOW REPROJECTION ERROR.  Vary deliberately:
      - tilt steeply, 30-45 degrees, in both axes
      - rotate in the image plane
      - fill the frame, then back off until the board is small
      - visit all four CORNERS of the frame, not just the middle
    This script refuses near-duplicate views so "I did not actually move it" is caught at capture
    time rather than discovered in the result.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

# ---- Board definition. Change these only if you print a different board. ---------------------
SQUARES_X, SQUARES_Y = 9, 7
SQUARE_LEN_MM, MARKER_LEN_MM = 30.0, 22.0
DICT = cv2.aruco.DICT_4X4_50

# ---- This camera, as Windows reports it: USB\VID_0C45&PID_6366 -------------------------------
# The FTC SDK matches a calibration entry by USB vendor/product id in DECIMAL, so the XML below
# carries 3141 / 25446. A vid/pid match is also the guard against the historical failure here: the
# old FX = 622.0 placeholder was not an optics mistake, it was the C920's 640x480 row copied from
# the SDK's table into a file describing a different camera.
ARDUCAM_VID, ARDUCAM_PID = 0x0C45, 0x6366

# ---- What fx SHOULD come out as, from the lens spec ------------------------------------------
# Arducam's B0332 datasheet: "Lens: FOV: 70(H), EFL: 2.8MM", 1/4 inch OV9281, 1280x800, 3um pixels.
# Two independent estimates, which agree to 2.1%:
#     from FOV:         fx = (1280/2) / tan(35 deg)      = 914.0
#     from EFL/pixel:   fx = 2.8 mm / 0.003 mm per px    = 933.3
# Expect a real calibration to land in 905-940 at 1280 wide. This is a PHYSICAL cross-check, and it
# catches what a reprojection error cannot: too few distinct poses gives a low RMS and a wrong fx.
NOMINAL_HFOV_DEG = 70.0
NOMINAL_EFL_MM = 2.8
PIXEL_SIZE_MM = 0.003
NATIVE_W, NATIVE_H = 1280, 800


def expected_fx(width):
    """fx implied by the datasheet FOV at this capture width.

    Valid for the NATIVE 16:10 modes. A 4:3 mode may CROP rather than scale, which changes the
    field of view and moves the principal point -- and crop-vs-scale is camera-specific and
    unpredictable. The SDK's own calibration table contains two cameras with opposite behaviour:
    the C920 crops horizontally across aspect ratios (HFOV 69.2 deg at 640x360 falling to 54.5 deg
    at 640x480, same width, fx differing by 1.34x), while the goBILDA camera preserves HFOV across
    all its modes and crops vertically. So do not carry a calibration across aspect ratios.
    """
    return (width / 2.0) / np.tan(np.radians(NOMINAL_HFOV_DEG / 2.0))


def get_board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN_MM, MARKER_LEN_MM, d)


def get_detector(board):
    cp = cv2.aruco.CharucoParameters()
    cp.minMarkers = 2
    dp = cv2.aruco.DetectorParameters()
    dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.CharucoDetector(board, cp, dp)


def make_board(outdir):
    os.makedirs(outdir, exist_ok=True)
    board = get_board()
    img = board.generateImage((int(SQUARES_X * SQUARE_LEN_MM * 10),
                               int(SQUARES_Y * SQUARE_LEN_MM * 10)))
    path = os.path.join(outdir, "charuco_board.png")
    cv2.imwrite(path, img)
    print("wrote %s" % path)
    print("  %dx%d squares, nominal %.1f mm square, %.1f mm marker, DICT_4X4_50"
          % (SQUARES_X, SQUARES_Y, SQUARE_LEN_MM, MARKER_LEN_MM))
    print("  image is %d x %d px for %.0f x %.0f mm, i.e. 10 px/mm at 100%% scale"
          % (img.shape[1], img.shape[0], SQUARES_X * SQUARE_LEN_MM, SQUARES_Y * SQUARE_LEN_MM))
    print()
    print("PRINT AT 100% SCALE. Then measure a square with calipers and pass --square-mm with")
    print("what you measured -- printers rescale and the measured value is the one that matters.")
    print("Tape it to glass or a clipboard: a bowed board is the #1 cause of a wrong fx.")


def open_camera(index, w, h):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit("Could not open camera %d. Close anything else using it, or try another index.\n"
                 "Run tools/vision/probe_camera.py to see which index is which." % index)

    # MJPG IS REQUIRED. In YUY2 -- which is the default almost everywhere, including VisionPortal --
    # this camera offers ONLY 1280x720 and 1280x800, and only at 10 fps. You cannot drop resolution
    # to escape it because YUY2 has no small modes here. USB 2.0 high-speed isochronous carries
    # about 24.6 MB/s and YUY2 at 1280x800 is 2,048,000 B/frame: a 12 fps ceiling. YUY2 spends 16
    # bits per pixel to carry an 8-bit mono sensor, so half the budget is null chroma.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    gw = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    gh = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if (gw, gh) != (w, h):
        cap.release()
        sys.exit("Camera substituted %dx%d for the requested %dx%d.\n"
                 "Calibrating at a resolution you did not ask for produces intrinsics for the\n"
                 "WRONG size, which is exactly the failure this script exists to prevent.\n"
                 "Known: 640x400 is NOT a real mode on this camera and comes back as 640x480."
                 % (gw, gh, w, h))
    return cap


def pose_is_new(corners, kept, min_shift_px):
    """Reject a view that is nearly a duplicate of one already kept.

    Compares mean corner displacement against every kept view. Crude, but it reliably catches the
    failure that matters: banking fifteen views that carry one view's worth of information.
    """
    c = corners.reshape(-1, 2)
    for k in kept:
        if k.shape != corners.shape:
            continue
        if np.mean(np.linalg.norm(c - k.reshape(-1, 2), axis=1)) < min_shift_px:
            return False
    return True


def capture(index, w, h, outdir, want, min_shift):
    os.makedirs(outdir, exist_ok=True)
    cap = open_camera(index, w, h)
    board = get_board()
    det = get_detector(board)

    existing = sorted(glob.glob(os.path.join(outdir, "shot_*.png")))
    n = len(existing)
    if n:
        print("found %d existing shots in %s -- continuing from there" % (n, outdir))

    kept = []
    print()
    print("Capturing %dx%d. SPACE keeps a view, Q quits. Target %d DIFFERENT views." % (w, h, want))
    print("Tilt steeply, rotate, near and far, and visit all four frame corners.")
    print()

    win = "charuco capture  --  SPACE=keep  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while True:
        ok, frame = cap.read()
        if not ok:
            print("dropped frame; camera may have been unplugged")
            break
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cc, ci, _, _ = det.detectBoard(gray)

        vis = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        count = 0 if cc is None else len(cc)
        fresh = count >= 12 and pose_is_new(cc, kept, min_shift)
        if count:
            cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)
        msg = ("%d corners - SPACE to keep" % count) if fresh else (
            "%d corners - too similar, MOVE IT" % count if count >= 12
            else "%d corners - need >=12" % count)
        colour = (0, 255, 0) if fresh else (0, 180, 255)
        cv2.putText(vis, "%d/%d kept" % (n, want), (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis, msg, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        cv2.imshow(win, vis)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q") or k == 27:
            break
        if k == ord(" ") and fresh:
            cv2.imwrite(os.path.join(outdir, "shot_%03d.png" % n), frame)
            kept.append(cc.copy())
            n += 1
            print("  kept %d/%d" % (n, want))
            if n >= want:
                print("got %d views -- now run with --solve" % n)
                break

    cap.release()
    cv2.destroyAllWindows()
    print("%d shots in %s" % (n, outdir))


def solve(w, h, srcdir, outdir, square_mm):
    global SQUARE_LEN_MM
    if square_mm is not None:
        SQUARE_LEN_MM = square_mm
    print("solving with square = %.2f mm%s" % (SQUARE_LEN_MM,
          "" if square_mm is not None else "  (NOMINAL -- measure it and pass --square-mm)"))

    board = get_board()
    det = get_detector(board)
    all_obj, all_img, used = [], [], []

    files = sorted(glob.glob(os.path.join(srcdir, "*.png"))) \
        + sorted(glob.glob(os.path.join(srcdir, "*.jpg")))
    if not files:
        sys.exit("no images in %s -- run --capture first" % srcdir)

    for f in files:
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if im is None:
            print("  skip (unreadable) %s" % f)
            continue
        if (im.shape[1], im.shape[0]) != (w, h):
            print("  skip (wrong size %dx%d) %s" % (im.shape[1], im.shape[0], f))
            continue
        cc, ci, _, _ = det.detectBoard(im)
        if cc is None or len(cc) < 12:
            print("  skip (too few corners) %s" % f)
            continue
        op, ip = board.matchImagePoints(cc, ci)
        all_obj.append(op)
        all_img.append(ip)
        used.append(f)

    if len(all_obj) < 8:
        sys.exit("only %d usable views -- take more (aim for 20)" % len(all_obj))

    # ZERO_TANGENT_DIST and FIX_K3: this is a low-distortion M12 lens (datasheet says <1%), and
    # fitting parameters the data cannot constrain lets them absorb error that belongs to fx. Fewer
    # free parameters gives a more trustworthy focal length here, which is the number we care about.
    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    rms, K, D, rvecs, tvecs, sdI, sdE, perView = cv2.calibrateCameraExtended(
        all_obj, all_img, (w, h), None, None, flags=flags)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = D.ravel()
    hfov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(h / (2 * fy)))
    exp_fx = expected_fx(w)
    off = 100.0 * (fx - exp_fx) / exp_fx

    print()
    print("=" * 78)
    print("CALIBRATION RESULT   %dx%d, %d views" % (w, h, len(used)))
    print("=" * 78)
    print("  fx fy            %.3f  %.3f   (fy/fx = %.4f)" % (fx, fy, fy / fx))
    print("  cx cy            %.3f  %.3f   (centre is %.1f, %.1f)" % (cx, cy, w / 2.0, h / 2.0))
    print("  1-sigma fx fy    %.3f  %.3f" % (sdI.ravel()[0], sdI.ravel()[1]))
    print("  1-sigma cx cy    %.3f  %.3f" % (sdI.ravel()[2], sdI.ravel()[3]))
    print("  distortion k1 k2 %+.5f %+.5f" % (d[0], d[1]))
    print("  overall RMS      %.4f px" % rms)
    print("  worst view       %.4f px   (%s)"
          % (float(perView.max()), used[int(perView.argmax())]))
    print("  implied FOV      %.1f deg H  x  %.1f deg V" % (hfov, vfov))
    print("  implied EFL      %.3f mm   (nominal %.1f mm at %.1f um pixels)"
          % (fx * PIXEL_SIZE_MM, NOMINAL_EFL_MM, PIXEL_SIZE_MM * 1000))
    print("  vs lens spec     %.0f expected from %.0f deg HFOV -> %+.1f%%"
          % (exp_fx, NOMINAL_HFOV_DEG, off))
    print()

    # ---- Sanity checks. A low RMS on a bad set of poses is the trap, so check the physics too. --
    problems = []
    if rms > 1.0:
        problems.append("RMS %.2f px is too high: bowed board, wrong --square-mm, motion blur, or "
                        "too few distinct poses." % rms)
    elif rms > 0.5:
        problems.append("RMS %.2f px is usable but not good; more varied poses would improve it."
                        % rms)
    if abs(fy / fx - 1.0) > 0.02:
        problems.append("fy/fx = %.4f. Square pixels should give ~1.000; this suggests a bad fit "
                        "or a non-square capture mode." % (fy / fx))
    if abs(cx - w / 2.0) > w * 0.08 or abs(cy - h / 2.0) > h * 0.08:
        problems.append("principal point (%.0f, %.0f) is >8%% off centre. Usually means the board "
                        "never reached the frame corners." % (cx, cy))
    if abs(off) > 15:
        problems.append("fx is %+.0f%% from the %.0f deg lens spec (expected ~%.0f). A reprojection "
                        "error can look fine while fx is wrong -- suspect the pose variety."
                        % (off, NOMINAL_HFOV_DEG, exp_fx))
    if float(perView.max()) > 3 * float(perView.mean()):
        problems.append("worst view is %.1fx the mean: delete %s and re-solve."
                        % (float(perView.max()) / float(perView.mean()),
                           used[int(perView.argmax())]))

    if problems:
        print("CHECK THESE:")
        for p in problems:
            print("  !  %s" % p)
    else:
        print("All sanity checks passed, including the physical fx-vs-lens-spec cross-check.")
    print()

    # ---- Bearing bias if distortion is ignored ----
    # The SDK's AprilTag path takes only the four pinhole terms, so this is the error you accept at
    # the frame edge by not undistorting yourself.
    xn, yn = (w - 1 - cx) / fx, (h - 1 - cy) / fy
    r2 = xn * xn + yn * yn
    scale = 1 + d[0] * r2 + d[1] * r2 * r2
    dpx = np.hypot(xn * fx, yn * fy) * abs(scale - 1)
    print("  Ignoring distortion costs %.2f px at the frame corner, i.e. %.3f deg of bearing bias."
          % (dpx, np.degrees(np.arctan(dpx / fx))))
    print("  The SDK's AprilTag path uses only fx/fy/cx/cy, so that is the error you accept unless")
    print("  you undistort before processing.")
    print()

    os.makedirs(outdir, exist_ok=True)

    java = """    // ---- Lens intrinsics: Arducam B0332 (OV9281 mono global shutter) + M12 lens ----
    //
    // MEASURED: %d ChArUco views at %dx%d, %.2f mm squares.
    // RMS %.4f px, worst view %.4f px. fy/fx = %.4f.
    // Implied FOV %.1f deg H x %.1f deg V; implied EFL %.3f mm (nominal %.1f).
    // Lens-spec cross-check: %+.1f%% from the %.0f deg HFOV prediction of %.0f.
    //
    // These REPLACE the Logitech C270 values (fx = 822.317), which belonged to a camera this team
    // does not own. Calibrated at %dx%d: do NOT carry them to another aspect ratio, because this
    // sensor may crop rather than scale.

    public static final int CAL_WIDTH  = %d;
    public static final int CAL_HEIGHT = %d;

    public static final double FX = %.3f;
    public static final double FY = %.3f;
    public static final double CX = %.3f;
    public static final double CY = %.3f;

    /** k1, k2, p1, p2, k3. The SDK's AprilTag path takes only the four above. */
    public static final double[] DISTORTION = { %.6f, %.6f, 0.0, 0.0, %.6f };
""" % (len(used), w, h, SQUARE_LEN_MM, rms, float(perView.max()), fy / fx,
       hfov, vfov, fx * PIXEL_SIZE_MM, NOMINAL_EFL_MM, off, NOMINAL_HFOV_DEG, exp_fx,
       w, h, w, h, fx, fy, cx, cy, d[0], d[1], d[4] if d.size > 4 else 0.0)

    xml = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Arducam B0332 (OV9281 mono global shutter). %d ChArUco views at %dx%d, RMS %.4f px.

  Install at TeamCode/src/main/res/xml/teamwebcamcalibrations.xml and rebuild.

  vid/pid are DECIMAL. This camera reports USB\\VID_0C45&PID_6366 on Windows,
  so vid="%d" pid="%d".

  VERIFY IT TOOK EFFECT. A calibration that silently fails to load is the failure mode that
  wastes a weekend: the SDK supplies a FAKE calibration rather than throwing, and every distance
  is then quietly wrong. MonoBallProcessor checks CameraCalibration.isFake() at init and reports
  it; T15 Mono Ball Test prints it. Expect fx near %.1f.
-->
<Calibrations>
    <Camera name="Arducam OV9281" vid="%d" pid="%d">
        <Calibration size="%d %d"
                     focalLength="%.3ff, %.3ff"
                     principalPoint="%.3ff, %.3ff"
                     distortionCoefficients="%.6f, %.6f, 0, 0, %.6f, 0, 0, 0"/>
    </Camera>
</Calibrations>
""" % (len(used), w, h, rms, ARDUCAM_VID, ARDUCAM_PID, fx,
       ARDUCAM_VID, ARDUCAM_PID, w, h, fx, fy, cx, cy,
       d[0], d[1], d[4] if d.size > 4 else 0.0)

    jp = os.path.join(outdir, "RobotConstants_Camera_snippet.java")
    xp = os.path.join(outdir, "teamwebcamcalibrations.xml")
    io_write(jp, java)
    io_write(xp, xml)
    np.savez(os.path.join(outdir, "calibration.npz"), K=K, D=D, size=(w, h), rms=rms,
             square_mm=SQUARE_LEN_MM, views=len(used))

    print("WROTE")
    print("  %s" % jp)
    print("  %s" % xp)
    print("  %s" % os.path.join(outdir, "calibration.npz"))
    print()
    print("ALSO: nothing in TeamCode currently calls AprilTagProcessor.setLensIntrinsics, so the")
    print("AprilTag pipeline is using the SDK's hardcoded fx = fy = 578.272 (a C920 entry). Either")
    print("install the XML above or pass these numbers explicitly:")
    print("    .setLensIntrinsics(%.3f, %.3f, %.3f, %.3f)" % (fx, fy, cx, cy))
    print()
    print(java)


def io_write(path, text):
    with open(path, "w") as f:
        f.write(text)


def selftest():
    """Recover KNOWN intrinsics from synthetic board views.

    This proves the modern CharucoDetector path works on this OpenCV build -- worth having, because
    the old interpolateCornersCharuco/calibrateCameraCharuco functions were removed in 4.7+ and
    most tutorials still use them, so "it does not work" is easy to misdiagnose as a setup problem.
    """
    w, h = NATIVE_W, NATIVE_H
    fx_true = fy_true = NOMINAL_EFL_MM / PIXEL_SIZE_MM     # 933.33
    cx_true, cy_true = w / 2 - 4.0, h / 2 + 3.0
    K_true = np.array([[fx_true, 0, cx_true], [0, fy_true, cy_true], [0, 0, 1]])
    D_true = np.zeros(5)

    board = get_board()
    det = get_detector(board)
    flat = board.generateImage((SQUARES_X * 120, SQUARES_Y * 120), marginSize=20)
    bh, bw = flat.shape
    mm_per_px = (SQUARES_X * SQUARE_LEN_MM) / bw

    all_obj, all_img = [], []
    rng = np.random.default_rng(0)
    for _ in range(24):
        rvec = np.array([rng.uniform(-0.45, 0.45), rng.uniform(-0.45, 0.45),
                         rng.uniform(-0.3, 0.3)])
        tvec = np.array([rng.uniform(-60, 60), rng.uniform(-40, 40), rng.uniform(420, 820)])
        src = np.float32([[0, 0], [bw, 0], [bw, bh], [0, bh]])
        c3d = np.float32([[0, 0, 0], [bw * mm_per_px, 0, 0],
                          [bw * mm_per_px, bh * mm_per_px, 0], [0, bh * mm_per_px, 0]])
        c3d[:, 0] -= bw * mm_per_px / 2
        c3d[:, 1] -= bh * mm_per_px / 2
        dst, _ = cv2.projectPoints(c3d, rvec, tvec, K_true, D_true)
        Hm = cv2.getPerspectiveTransform(src, dst.reshape(-1, 2).astype(np.float32))
        img = cv2.warpPerspective(flat, Hm, (w, h), borderValue=255, flags=cv2.INTER_LINEAR)
        cc, ci, _, _ = det.detectBoard(img)
        if cc is None or len(cc) < 12:
            continue
        op, ip = board.matchImagePoints(cc, ci)
        all_obj.append(op)
        all_img.append(ip)

    print("selftest: %d synthetic views detected" % len(all_obj))
    if len(all_obj) < 8:
        print("SELFTEST INCONCLUSIVE -- the detector found too few views")
        return 1

    rms, K, D, _, _ = cv2.calibrateCamera(
        all_obj, all_img, (w, h), None, None,
        flags=cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    efx = 100 * (K[0, 0] / fx_true - 1)
    efy = 100 * (K[1, 1] / fy_true - 1)
    print("  rms %.4f px" % rms)
    print("  fx  true %.2f -> %.2f   (%+.2f%%)" % (fx_true, K[0, 0], efx))
    print("  fy  true %.2f -> %.2f   (%+.2f%%)" % (fy_true, K[1, 1], efy))
    print("  cx  true %.2f -> %.2f" % (cx_true, K[0, 2]))
    print("  cy  true %.2f -> %.2f" % (cy_true, K[1, 2]))
    ok = abs(efx) < 1.0 and abs(efy) < 1.0 and rms < 0.5
    print()
    print("  %s" % ("PASS -- the CharucoDetector path works on this OpenCV build."
                    if ok else "FAIL -- recovery is off by more than 1%; do not trust real runs."))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description="Calibrate the Arducam B0332 and emit FTC-ready intrinsics.")
    ap.add_argument("--make-board", action="store_true", help="write a printable ChArUco board")
    ap.add_argument("--capture", action="store_true", help="collect views from the camera")
    ap.add_argument("--solve", action="store_true", help="fit intrinsics from collected views")
    ap.add_argument("--selftest", action="store_true", help="verify the method, no camera needed")
    ap.add_argument("--camera", type=int, default=1, help="camera index (the Arducam probed as 1)")
    ap.add_argument("--width", type=int, default=NATIVE_W, help="capture width")
    ap.add_argument("--height", type=int, default=NATIVE_H, help="capture height")
    ap.add_argument("--square-mm", type=float, default=None,
                    help="MEASURED square size; printers rescale, so measure it")
    ap.add_argument("--views", type=int, default=20, help="views to collect")
    ap.add_argument("--min-shift", type=float, default=18.0,
                    help="mean corner shift in px for a view to count as new")
    ap.add_argument("--shots", default="calibration/shots", help="where views are stored")
    ap.add_argument("--outdir", default="calibration", help="where results are written")
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())
    if a.make_board:
        make_board(a.outdir)
    elif a.capture:
        capture(a.camera, a.width, a.height, a.shots, a.views, a.min_shift)
    elif a.solve:
        solve(a.width, a.height, a.shots, a.outdir, a.square_mm)
    else:
        ap.print_help()
        print()
        print("Typical first run:")
        print("  python tools/vision/calibrate.py --selftest")
        print("  python tools/vision/calibrate.py --make-board      # then print it at 100%")
        print("  python tools/vision/calibrate.py --capture --camera 1 --square-mm 29.8")
        print("  python tools/vision/calibrate.py --solve --square-mm 29.8")


if __name__ == "__main__":
    main()
