#!/usr/bin/env python3
"""
Verify the Java BallGeometry matches the Python ball_geometry, value for value.

WHY THIS EXISTS
    The detector is prototyped in Python (fast to iterate, easy to test against synthetic ground
    truth) and then ported to Java to run on the Control Hub.  A port is where silent divergence
    lives: a sign flipped, a rotation applied in the wrong order, degrees left where radians were
    meant, a null handled differently at the horizon.  None of those throw.  All of them produce
    plausible numbers that are simply not the numbers the Python version was validated with -- so
    the Python test suite keeps passing while the robot aims somewhere else.

    So instead of trusting the port: the Java prints its own answers for a few thousand input
    combinations, this script recomputes every one from the Python reference, and they are compared
    to twelve significant figures.

HOW TO RUN
    1) javac -d <classes> TeamCode/.../vision/BallGeometry.java <scratch>/CrossCheck.java
    2) java -cp <classes> CrossCheck > vectors.csv
    3) python tools/vision/crosscheck_java.py vectors.csv

    run_crosscheck.sh does all three.

WHAT COUNTS AS AGREEMENT
    Relative difference below 1e-9, or both sides NaN, or both sides infinite with the same sign.
    NaN-vs-number is reported as a mismatch even though both "look" like a non-answer: the two
    implementations disagreeing about whether a pixel is above the horizon is exactly the kind of
    boundary bug this is here to catch.
"""

import math
import sys

from ball_geometry import (
    CameraPose, Intrinsics,
    ground_plane_position, apparent_size_slant_distance, apparent_size_ground_range,
    predicted_radius_px, classify_by_radius, consistency_ratio, range_error_per_pixel,
)

INTR = Intrinsics(fx=457.0, fy=457.0, cx=320.0, cy=200.0, width=640, height=400)
DIAMETERS = [2.8, 3.6]
CANDIDATES = [("D0", 2.8), ("D1", 3.6)]

# Must mirror CrossCheck.java's pose list exactly, in the same order.
POSES = [
    CameraPose(14.0, 0.0),
    CameraPose(14.0, 10.0),
    CameraPose(14.0, 18.0),
    CameraPose(14.0, 25.0, x_in=5.0, y_in=2.0),
    CameraPose(9.5, 30.0, x_in=-3.0, y_in=1.5),
    CameraPose(14.0, 15.0, x_in=4.0, y_in=0.0, yaw_rad=math.radians(30),
               pivot_x_in=3.0, pivot_y_in=0.0, on_turret=True),
    CameraPose(14.0, 15.0, x_in=4.0, y_in=0.0, yaw_rad=math.radians(-75),
               pivot_x_in=3.0, pivot_y_in=0.0, on_turret=True),
    CameraPose(16.0, 22.0, x_in=2.0, y_in=-1.0, yaw_rad=math.radians(120),
               pivot_x_in=-2.0, pivot_y_in=1.0, on_turret=True),
]

TOL = 1e-9


def parse(tok):
    t = tok.strip().lower()
    if t == "nan":
        return float("nan")
    if t == "inf":
        return float("inf")
    if t == "-inf":
        return float("-inf")
    return float(t)


def agree(a, b):
    an, bn = isinstance(a, float) and math.isnan(a), isinstance(b, float) and math.isnan(b)
    if an or bn:
        return an and bn
    ai, bi = math.isinf(a), math.isinf(b)
    if ai or bi:
        return ai and bi and (a > 0) == (b > 0)
    scale = max(1.0, abs(a), abs(b))
    return abs(a - b) / scale < TOL


def horizon_row(intr, pose):
    return intr.cy - intr.fy * math.tan(pose.pitch_rad)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: crosscheck_java.py <vectors.csv from CrossCheck>")

    with open(sys.argv[1]) as f:
        lines = [ln for ln in f.read().splitlines() if ln.strip()]

    header = lines[0].split(",")
    want = ["case", "pose", "u", "v", "radius", "D", "horizon", "gp_x", "gp_y", "gp_range",
            "sz_slant", "sz_range", "pred_r", "consistency", "cls_idx", "cls_range",
            "cls_pred", "cls_sep", "err_gp", "err_sz"]
    if header != want:
        sys.exit("unexpected header:\n  got  %s\n  want %s" % (header, want))

    n = 0
    mismatches = []
    fields_checked = 0

    for ln in lines[1:]:
        p = ln.split(",")
        case = int(p[0])
        pose_i = int(p[1])
        u, v, r, D = parse(p[2]), parse(p[3]), parse(p[4]), parse(p[5])
        pose = POSES[pose_i]

        # --- recompute everything from the Python reference ---
        py = {}
        py["horizon"] = horizon_row(INTR, pose)

        gp = ground_plane_position(u, v, INTR, pose, D)
        py["gp_x"] = gp[0] if gp else float("nan")
        py["gp_y"] = gp[1] if gp else float("nan")
        py["gp_range"] = gp[2] if gp else float("nan")

        slant = apparent_size_slant_distance(r, INTR, D)
        py["sz_slant"] = slant if slant is not None else float("nan")
        szr = apparent_size_ground_range(r, INTR, pose, D)
        py["sz_range"] = szr if szr is not None else float("nan")

        py["pred_r"] = (predicted_radius_px(gp[2], INTR, pose, D) if gp else float("nan"))
        if py["pred_r"] is None:
            py["pred_r"] = float("nan")

        cons = consistency_ratio(u, v, r, INTR, pose, D)
        py["consistency"] = cons[2] if cons else float("nan")

        cls = classify_by_radius(u, v, r, INTR, pose, CANDIDATES)
        if cls is None:
            py["cls_idx"] = -1.0
            py["cls_range"] = float("nan")
            py["cls_pred"] = float("nan")
            py["cls_sep"] = float("nan")
        else:
            py["cls_idx"] = float(0 if cls["type"] == "D0" else 1)
            py["cls_range"] = cls["range_in"]
            py["cls_pred"] = cls["predicted_radius_px"]
            py["cls_sep"] = cls["separation_px"]

        egp, esz = range_error_per_pixel(gp[2] if gp else 48.0, INTR, pose, D)
        py["err_gp"], py["err_sz"] = egp, esz

        # --- compare ---
        for idx, name in enumerate(want):
            if idx < 6:
                continue
            java_val = parse(p[idx])
            py_val = py[name]
            fields_checked += 1
            if not agree(java_val, py_val):
                mismatches.append((case, pose_i, u, v, r, D, name, java_val, py_val))
        n += 1

    print()
    print("=" * 78)
    print("Java BallGeometry vs Python ball_geometry")
    print("=" * 78)
    print("  %d cases, %d field comparisons, tolerance %g relative" % (n, fields_checked, TOL))
    print()

    if not mismatches:
        print("  AGREE on every field of every case.")
        print()
        print("  The Java port is numerically identical to the Python that was validated against")
        print("  synthetic ground truth, so the Python test results carry over to the robot code.")
        print("=" * 78)
        return 0

    print("  %d MISMATCHES (showing up to 25):" % len(mismatches))
    print()
    print("  %-6s %-5s %-7s %-7s %-7s %-5s %-13s %-18s %s"
          % ("case", "pose", "u", "v", "r", "D", "field", "java", "python"))
    for m in mismatches[:25]:
        case, pi, u, v, r, D, name, jv, pv = m
        print("  %-6d %-5d %-7.1f %-7.1f %-7.1f %-5.1f %-13s %-18.10g %.10g"
              % (case, pi, u, v, r, D, name, jv, pv))

    by_field = {}
    for m in mismatches:
        by_field[m[6]] = by_field.get(m[6], 0) + 1
    print()
    print("  mismatches by field: %s"
          % ", ".join("%s=%d" % kv for kv in sorted(by_field.items(), key=lambda k: -k[1])))
    print("=" * 78)
    return 1


if __name__ == "__main__":
    sys.exit(main())
