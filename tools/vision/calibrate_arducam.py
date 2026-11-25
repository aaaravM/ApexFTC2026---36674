#!/usr/bin/env python3
"""
Arducam B0332 (OV9281 mono global shutter) intrinsics calibration for FTC.

Verified against OpenCV 4.12.0 / Python 3.11 on Windows.
Uses the MODERN aruco API (cv2.aruco.CharucoDetector). The old
cv2.aruco.interpolateCornersCharuco / calibrateCameraCharuco were REMOVED
in OpenCV 4.7+ and do not exist in 4.12 -- do not copy old tutorials.

USAGE
  1) python calib_charuco.py --make-board           # writes charuco_board.png -> print it
  2) python calib_charuco.py --capture --w 1280 --h 800
  3) python calib_charuco.py --solve  --w 1280 --h 800
"""
import argparse, glob, os, sys
import cv2
import numpy as np

# ---- Board definition. 9x7 squares, 30 mm square, 22 mm marker, DICT_4X4_50.
SQUARES_X, SQUARES_Y = 9, 7
SQUARE_LEN_MM, MARKER_LEN_MM = 30.0, 22.0
DICT = cv2.aruco.DICT_4X4_50


def make_board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN_MM, MARKER_LEN_MM, d)
    # 10 px per mm -> exact print scale check: image is 2700 x 2100 px for 270 x 210 mm
    img = board.generateImage((int(SQUARES_X * SQUARE_LEN_MM * 10), int(SQUARES_Y * SQUARE_LEN_MM * 10)))
    cv2.imwrite("charuco_board.png", img)
    print("wrote charuco_board.png -- print at 100%% scale, then MEASURE a square with calipers")
    print("and set SQUARE_LEN_MM to what you measured. Tape it to GLASS or a clipboard: a bowed")
    print("board is the #1 cause of a wrong fx.")
    return board


def get_board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN_MM, MARKER_LEN_MM, d)


def get_detector(board):
    cp = cv2.aruco.CharucoParameters()
    cp.minMarkers = 2
    dp = cv2.aruco.DetectorParameters()
    dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.CharucoDetector(board, cp, dp)


def capture(w, h, out="shots"):
    os.makedirs(out, exist_ok=True)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    # MJPG is REQUIRED: the B0332 only offers YUY2 at 10 fps.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    print("actual:", cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("SPACE = save, q = quit. Get >=20 shots. See the pose list in the writeup.")
    board, det, n = get_board(), get_detector(get_board()), 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vis = frame.copy()
        g = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cc, ci, _, _ = det.detectBoard(g)
        if cc is not None and len(cc) > 0:
            cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)
            cv2.putText(vis, f"{len(cc)} corners  saved={n}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("capture", vis)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" "):
            cv2.imwrite(f"{out}/shot_{n:03d}.png", frame)
            n += 1
            print("saved", n)
        elif k == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def solve(w, h, src="shots"):
    board, det = get_board(), get_detector(get_board())
    all_obj, all_img, used = [], [], []
    for f in sorted(glob.glob(f"{src}/*.png")) + sorted(glob.glob(f"{src}/*.jpg")):
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if im is None or (im.shape[1], im.shape[0]) != (w, h):
            print("skip (wrong size)", f, None if im is None else im.shape)
            continue
        cc, ci, _, _ = det.detectBoard(im)
        if cc is None or len(cc) < 12:          # need >=12 corners for a useful view
            print("skip (too few corners)", f)
            continue
        op, ip = board.matchImagePoints(cc, ci)
        all_obj.append(op); all_img.append(ip); used.append(f)
    if len(all_obj) < 8:
        print(f"only {len(all_obj)} usable views -- take more"); return

    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    rms, K, D, rvecs, tvecs, sdI, sdE, perView = cv2.calibrateCameraExtended(
        all_obj, all_img, (w, h), None, None, flags=flags)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"\nviews used        {len(used)}")
    print(f"overall RMS       {rms:.4f} px   (want < 0.5; < 0.3 is good)")
    print(f"worst view        {float(perView.max()):.4f} px  ({used[int(perView.argmax())]})")
    print(f"fx fy             {fx:.3f}  {fy:.3f}   (fy/fx = {fy/fx:.4f}, want within 1%)")
    print(f"cx cy             {cx:.3f}  {cy:.3f}   (want within ~5%% of {w/2:.1f}, {h/2:.1f})")
    print(f"1-sigma fx fy     {sdI.ravel()[0]:.3f}  {sdI.ravel()[1]:.3f}")
    print(f"1-sigma cx cy     {sdI.ravel()[2]:.3f}  {sdI.ravel()[3]:.3f}")
    print(f"distortion k1 k2  {D.ravel()[0]:+.5f} {D.ravel()[1]:+.5f}")
    hfov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(h / (2 * fy)))
    print(f"implied FOV       {hfov:.1f} deg H  x  {vfov:.1f} deg V")
    print(f"implied EFL       {fx * 0.003:.3f} mm  (3.0 um pixels; nominal lens is 2.8 mm)")

    # worst-case radial error at the frame corner, in pixels, if you ignore distortion
    k1, k2 = float(D.ravel()[0]), float(D.ravel()[1])
    xn = (w - 1 - cx) / fx; yn = (h - 1 - cy) / fy
    r2 = xn * xn + yn * yn
    scale = 1 + k1 * r2 + k2 * r2 * r2
    dpx = np.hypot(xn * fx, yn * fy) * abs(scale - 1)
    print(f"\ncorner radial error if distortion ignored: {dpx:.2f} px")
    print(f"  -> as an angle: {np.degrees(np.arctan(dpx / fx)):.3f} deg of bearing bias at the frame edge")

    print(f"""
--- paste into TeamCode/src/main/res/xml/teamwebcamcalibrations.xml ---
    <Camera vid="Arducam" pid="0xYOUR_PID">
        <Calibration
            size="{w} {h}"
            focalLength="{fx:.3f}f, {fy:.3f}f"
            principalPoint="{cx:.3f}f, {cy:.3f}f"
            distortionCoefficients="{D.ravel()[0]:.6f}, {D.ravel()[1]:.6f}, 0, 0, {D.ravel()[4] if D.size > 4 else 0:.6f}, 0, 0, 0"
            />
    </Camera>

--- or explicitly in Java (this always wins over the XML) ---
    .setLensIntrinsics({fx:.3f}, {fy:.3f}, {cx:.3f}, {cy:.3f})
""")
    np.savez("arducam_intrinsics.npz", K=K, D=D, size=(w, h), rms=rms)


def selftest():
    """Render synthetic views of the board and recover known intrinsics.
    This proves the API calls above work on this OpenCV build."""
    w, h = 1280, 800
    fx_true = fy_true = 933.33
    cx_true, cy_true = w / 2 - 4.0, h / 2 + 3.0
    K_true = np.array([[fx_true, 0, cx_true], [0, fy_true, cy_true], [0, 0, 1]])
    D_true = np.zeros(5)
    board = get_board()
    det = get_detector(board)
    flat = board.generateImage((SQUARES_X * 120, SQUARES_Y * 120), marginSize=20)
    bh, bw = flat.shape
    # board plane: map image pixels -> mm
    mm_per_px = (SQUARES_X * SQUARE_LEN_MM) / bw
    all_obj, all_img = [], []
    rng = np.random.default_rng(0)
    for i in range(24):
        rvec = np.array([rng.uniform(-0.45, 0.45), rng.uniform(-0.45, 0.45), rng.uniform(-0.3, 0.3)])
        tvec = np.array([rng.uniform(-60, 60), rng.uniform(-40, 40), rng.uniform(420, 820)])
        src = np.float32([[0, 0], [bw, 0], [bw, bh], [0, bh]])
        corners3d = np.float32([[0, 0, 0], [bw * mm_per_px, 0, 0],
                                [bw * mm_per_px, bh * mm_per_px, 0], [0, bh * mm_per_px, 0]])
        corners3d[:, 0] -= bw * mm_per_px / 2
        corners3d[:, 1] -= bh * mm_per_px / 2
        dst, _ = cv2.projectPoints(corners3d, rvec, tvec, K_true, D_true)
        Hm = cv2.getPerspectiveTransform(src, dst.reshape(-1, 2).astype(np.float32))
        img = cv2.warpPerspective(flat, Hm, (w, h), borderValue=255, flags=cv2.INTER_LINEAR)
        cc, ci, _, _ = det.detectBoard(img)
        if cc is None or len(cc) < 12:
            continue
        op, ip = board.matchImagePoints(cc, ci)
        all_obj.append(op); all_img.append(ip)
    print(f"selftest: {len(all_obj)} synthetic views detected")
    if len(all_obj) < 8:
        print("SELFTEST INCONCLUSIVE"); return
    rms, K, D, _, _ = cv2.calibrateCamera(all_obj, all_img, (w, h), None, None,
                                          flags=cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    print(f"selftest rms={rms:.4f}")
    print(f"  fx true {fx_true:.2f} -> recovered {K[0,0]:.2f}   err {100*(K[0,0]/fx_true-1):+.2f}%")
    print(f"  fy true {fy_true:.2f} -> recovered {K[1,1]:.2f}   err {100*(K[1,1]/fy_true-1):+.2f}%")
    print(f"  cx true {cx_true:.2f} -> recovered {K[0,2]:.2f}")
    print(f"  cy true {cy_true:.2f} -> recovered {K[1,2]:.2f}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--make-board", action="store_true")
    p.add_argument("--capture", action="store_true")
    p.add_argument("--solve", action="store_true")
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--w", type=int, default=1280)
    p.add_argument("--h", type=int, default=800)
    a = p.parse_args()
    if a.make_board: make_board()
    elif a.capture: capture(a.w, a.h)
    elif a.solve: solve(a.w, a.h)
    elif a.selftest: selftest()
    else: p.print_help()
