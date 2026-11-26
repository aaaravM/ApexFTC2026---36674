#!/usr/bin/env python3
"""
Validate ball_geometry.py against synthetic ground truth.  No camera, no OpenCV, no robot.

WHY A ROUND-TRIP TEST
    Projection maths is easy to get subtly wrong -- a sign on the pitch, a swapped axis, yaw applied
    in the wrong order -- and every one of those errors produces plausible-looking numbers that are
    simply untrue.  Eyeballing the output tells you nothing.

    So: put a ball at a KNOWN robot-frame position, forward-project it to a pixel and an apparent
    radius, then feed that pixel and radius back through the inverse and check we recover the
    position we started from.  If the loop closes to a thousandth of an inch across a spread of
    poses, yaws and pitches, the transform chain is right.  If a sign is wrong, it will not close.

    The forward projection here is written INDEPENDENTLY of the inverse -- it is derived from the
    camera basis directly rather than by rearranging the inverse -- so this is a real check and not
    a tautology.

RUN
    python tools/vision/test_geometry.py
"""

import math
import sys

from ball_geometry import (
    CameraPose, Intrinsics,
    ground_plane_position, apparent_size_ground_range, apparent_size_slant_distance,
    predicted_radius_px, classify_by_radius, consistency_ratio,
    range_error_per_pixel, robot_to_field, _ray_camera_to_world,
)

POLLEN_D = 2.8
NECTAR_D = 3.6
CANDIDATES = [("POLLEN", POLLEN_D), ("NECTAR", NECTAR_D)]

# PROVISIONAL intrinsics: the Arducam has NOT been calibrated yet. These assume a 70 deg horizontal
# field of view at 640 wide (fx = 320 / tan(35 deg) = 457.0), which is a plausible M12 lens but a
# GUESS. Every absolute distance printed below moves proportionally with fx. The round-trip tests do
# not care -- they pass for any self-consistent intrinsics, which is the point of testing this way.
PROVISIONAL = Intrinsics(fx=457.0, fy=457.0, cx=320.0, cy=200.0, width=640, height=400)

PASS, FAIL = 0, 0


def check(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        print("  FAIL  %s   %s" % (name, detail))


def project(bx, by, intr, pose, D):
    """Forward: a ball centre at robot-frame (bx, by, D/2) -> (u, v, radius_px).

    Derived independently from the camera basis, expressed in the yaw-free world frame:
        x_cam (right)   = ( 0,      -1,  0      )
        y_cam (down)    = (-sin p,   0, -cos p  )
        z_cam (forward) = ( cos p,   0, -sin p  )
    Returns None if the ball is behind the image plane.
    """
    lx, ly = pose.lens_in_robot_frame()
    wx, wy, wz = bx - lx, by - ly, D / 2.0 - pose.height_in

    # Undo the lens yaw: express the vector in the camera's own (yaw-free) heading.
    cy_, sy_ = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    w1x = wx * cy_ + wy * sy_
    w1y = -wx * sy_ + wy * cy_
    w1z = wz

    sp, cp = math.sin(pose.pitch_rad), math.cos(pose.pitch_rad)
    vx = -w1y
    vy = -w1x * sp - w1z * cp
    vz = w1x * cp - w1z * sp

    if vz <= 1e-9:
        return None
    u = intr.cx + intr.fx * vx / vz
    v = intr.cy + intr.fy * vy / vz
    L = math.sqrt(vx * vx + vy * vy + vz * vz)
    r = intr.fx * (D / 2.0) / L
    return u, v, r


# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("1. Camera basis is orthonormal and right-handed")
print("=" * 78)

for pitch in (0.0, 10.0, 25.0, 40.0):
    p = math.radians(pitch)
    x_b = (0.0, -1.0, 0.0)
    y_b = (-math.sin(p), 0.0, -math.cos(p))
    z_b = (math.cos(p), 0.0, -math.sin(p))

    def dot(a, b):
        return sum(ai * bi for ai, bi in zip(a, b))

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    unit = all(abs(dot(v, v) - 1.0) < 1e-12 for v in (x_b, y_b, z_b))
    orth = (abs(dot(x_b, y_b)) < 1e-12 and abs(dot(y_b, z_b)) < 1e-12 and abs(dot(x_b, z_b)) < 1e-12)
    rh = max(abs(a - b) for a, b in zip(cross(x_b, y_b), z_b)) < 1e-12
    check("pitch %4.0f deg: unit, orthogonal, right-handed" % pitch, unit and orth and rh)

# The ray helper must agree with the basis it claims to use.
for (a, b, pitch) in [(0.0, 0.0, 0.0), (0.3, 0.2, 15.0), (-0.5, 0.4, 30.0)]:
    p = math.radians(pitch)
    got = _ray_camera_to_world(a, b, p)
    want = (a * 0.0 + b * (-math.sin(p)) + math.cos(p),
            a * (-1.0) + b * 0.0 + 0.0,
            a * 0.0 + b * (-math.cos(p)) + (-math.sin(p)))
    check("_ray_camera_to_world matches basis (a=%.1f b=%.1f p=%.0f)" % (a, b, pitch),
          max(abs(g - w) for g, w in zip(got, want)) < 1e-12)

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("2. ROUND TRIP: known position -> pixel -> recovered position")
print("=" * 78)

POSES = [
    ("chassis, level, 14in up", CameraPose(14.0, 0.0)),
    ("chassis, 10deg down", CameraPose(14.0, 10.0)),
    ("chassis, 25deg down", CameraPose(14.0, 25.0)),
    ("chassis, offset fwd/left", CameraPose(14.0, 15.0, x_in=5.0, y_in=2.0)),
    ("turret yaw +30deg", CameraPose(14.0, 15.0, x_in=4.0, y_in=0.0,
                                     yaw_rad=math.radians(30), pivot_x_in=3.0,
                                     pivot_y_in=0.0, on_turret=True)),
    ("turret yaw -75deg", CameraPose(14.0, 15.0, x_in=4.0, y_in=0.0,
                                     yaw_rad=math.radians(-75), pivot_x_in=3.0,
                                     pivot_y_in=0.0, on_turret=True)),
]

TARGETS = [(24.0, 0.0), (36.0, 12.0), (60.0, -18.0), (96.0, 6.0), (30.0, -30.0)]

worst = 0.0
for label, pose in POSES:
    for (bx, by) in TARGETS:
        # Place the target in front of the LENS direction so it is actually in frame.
        c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        tx = pose.pivot_x_in + bx * c - by * s if pose.on_turret else bx
        ty = pose.pivot_y_in + bx * s + by * c if pose.on_turret else by

        proj = project(tx, ty, PROVISIONAL, pose, POLLEN_D)
        if proj is None:
            continue
        u, v, r = proj
        got = ground_plane_position(u, v, PROVISIONAL, pose, POLLEN_D)
        if got is None:
            check("%s @ (%.0f,%.0f): ground plane returned None" % (label, tx, ty), False,
                  "pixel (%.1f,%.1f) treated as above horizon" % (u, v))
            continue
        gx, gy, _ = got
        err = math.hypot(gx - tx, gy - ty)
        worst = max(worst, err)
        if err > 1e-6:
            check("%s @ (%.1f,%.1f)" % (label, tx, ty), False,
                  "recovered (%.4f,%.4f), error %.6f in" % (gx, gy, err))

check("all %d pose/target round trips recover position (worst error %.2e in)"
      % (len(POSES) * len(TARGETS), worst), worst < 1e-6)

# Apparent size should recover the slant distance too (small-angle approx, so allow a little).
pose = CameraPose(14.0, 15.0)
sz_worst = 0.0
for (bx, by) in TARGETS:
    u, v, r = project(bx, by, PROVISIONAL, pose, POLLEN_D)
    true_rng = math.hypot(bx, by)
    got = apparent_size_ground_range(r, PROVISIONAL, pose, POLLEN_D)
    sz_worst = max(sz_worst, abs(got - true_rng) / true_rng)
check("apparent-size range recovers truth within 0.1%% (worst %.4f%%)" % (100 * sz_worst),
      sz_worst < 1e-3)

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("3. Horizon test rejects anything that cannot be on the floor")
print("=" * 78)

pose = CameraPose(14.0, 12.0)
# The horizon row: where the ray is exactly level. From rz = -sin p - b cos p = 0 -> b = -tan p.
horizon_v = PROVISIONAL.cy + PROVISIONAL.fy * (-math.tan(pose.pitch_rad))
print("  pitch 12 deg, 640x400 -> horizon at image row v = %.1f" % horizon_v)

check("row well above horizon -> None",
      ground_plane_position(320, horizon_v - 40, PROVISIONAL, pose, POLLEN_D) is None)
check("row exactly at horizon -> None",
      ground_plane_position(320, horizon_v, PROVISIONAL, pose, POLLEN_D) is None)
below = ground_plane_position(320, horizon_v + 40, PROVISIONAL, pose, POLLEN_D)
check("row below horizon -> a position", below is not None,
      "" if below else "should have intersected the floor")

# Rows just below the horizon give enormous ranges: that is real, and is why a max-range gate is
# needed as well as the horizon test.
for dv in (2, 5, 20, 100):
    gp = ground_plane_position(320, horizon_v + dv, PROVISIONAL, pose, POLLEN_D)
    if gp:
        print("    %3d px below horizon -> range %8.1f in" % (dv, gp[2]))

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("4. Ball type from size alone (no colour), and where that stops working")
print("=" * 78)

pose = CameraPose(14.0, 15.0)
print("  %-8s %-10s %-12s %-12s %-10s %s"
      % ("truth", "range in", "r_px", "classified", "sep px", "trust?"))
mis = 0
for D, name in ((POLLEN_D, "POLLEN"), (NECTAR_D, "NECTAR")):
    for rng in (24, 36, 48, 72, 96, 120, 144):
        u, v, r = project(rng, 0.0, PROVISIONAL, pose, D)
        res = classify_by_radius(u, v, r, PROVISIONAL, pose, CANDIDATES)
        if res is None:
            continue
        ok = res["type"] == name
        if not ok and res["type_trustworthy"]:
            mis += 1
        print("  %-8s %-10.0f %-12.2f %-12s %-10.2f %s%s"
              % (name, rng, r, res["type"], res["separation_px"],
                 "yes" if res["type_trustworthy"] else "NO - use colour sensor",
                 "" if ok else "   <-- MISCLASSIFIED"))

check("no misclassification while separation is still trustworthy", mis == 0,
      "%d trustworthy-but-wrong classifications" % mis)

# Find the range at which size can no longer tell the two apart.
limit = None
for rng in range(12, 400):
    u, v, r = project(float(rng), 0.0, PROVISIONAL, pose, POLLEN_D)
    res = classify_by_radius(u, v, r, PROVISIONAL, pose, CANDIDATES)
    if res and not res["type_trustworthy"]:
        limit = rng
        break
print()
print("  POLLEN/NECTAR become indistinguishable by size beyond about %s in"
      % (limit if limit else ">400"))
print("  (separation < 1.5 px). Past that, the intake colour sensor is the only ball-type source.")
print("  NOTE: scales with fx, so recalculate once the camera is actually calibrated.")

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("5. Which range method is more accurate, and by how much")
print("=" * 78)

pose = CameraPose(14.0, 15.0)
print("  Inches of range error per PIXEL of measurement error, POLLEN (2.8 in):")
print("  %-10s %-16s %-16s %s" % ("range in", "ground plane", "apparent size", "ground is better by"))
for rng in (24, 48, 72, 96, 144):
    gp, sz = range_error_per_pixel(rng, PROVISIONAL, pose, POLLEN_D)
    print("  %-10.0f %-16.3f %-16.3f %.1fx" % (rng, gp, sz, sz / gp))

gp24, sz24 = range_error_per_pixel(24, PROVISIONAL, pose, POLLEN_D)
gp96, sz96 = range_error_per_pixel(96, PROVISIONAL, pose, POLLEN_D)
check("ground plane beats apparent size at close range", sz24 > gp24)
check("ground plane beats apparent size at long range", sz96 > gp96)
print()
print("  The ratio is H/(D/2) -- camera height above the ball centre, over the ball radius.")
print("  It does not depend on range, which is why the ground plane wins everywhere, and why")
print("  MOUNTING THE CAMERA HIGHER directly buys range accuracy. That is a free design win.")

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("6. Consistency check flags an airborne ball")
print("=" * 78)

pose = CameraPose(14.0, 15.0)
u, v, r = project(48.0, 0.0, PROVISIONAL, pose, POLLEN_D)
c = consistency_ratio(u, v, r, PROVISIONAL, pose, POLLEN_D)
check("floor ball: both methods agree within 1%%", c is not None and abs(c[2] - 1.0) < 0.01,
      "ratio %.4f" % c[2] if c else "None")

# An airborne ball at the same pixel is FURTHER away than the floor assumption implies, so it looks
# smaller than a floor ball at that pixel would -- the two methods diverge, which is the signal.
u2, v2, _ = project(48.0, 0.0, PROVISIONAL, CameraPose(14.0 - 10.0, 15.0), POLLEN_D)
r_far = PROVISIONAL.fx * (POLLEN_D / 2.0) / 90.0   # radius as if 90 in away
c2 = consistency_ratio(u, v, r_far, PROVISIONAL, pose, POLLEN_D)
check("ball much smaller than the floor model predicts -> ratio far from 1",
      c2 is not None and c2[2] > 1.3, "ratio %.3f" % c2[2] if c2 else "None")
if c2:
    print("    floor model says %.1f in, apparent size says %.1f in, ratio %.2f"
          % (c2[0], c2[1], c2[2]))
    print("    -> either airborne, or not a ball. Both worth knowing.")

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("7. Robot frame -> field frame")
print("=" * 78)

# A ball 24 in ahead of a robot at the origin facing +X is at field (24, 0).
check("heading 0", max(abs(a - b) for a, b in zip(robot_to_field(24, 0, 0, 0, 0), (24, 0))) < 1e-9)
# Facing +Y (90 deg CCW), 24 in "ahead" is field +Y.
check("heading 90 deg",
      max(abs(a - b) for a, b in zip(robot_to_field(24, 0, 0, 0, math.pi / 2), (0, 24))) < 1e-9)
# Offset robot.
check("offset robot, heading 180 deg",
      max(abs(a - b) for a, b in zip(robot_to_field(10, 0, 50, 50, math.pi), (40, 50))) < 1e-9)

# What stale-pose latency costs.
print()
for speed, lat in ((50, 0.100), (50, 0.030), (50, 0.010)):
    print("  robot at %d in/s, %3.0f ms latency -> %.2f in of ball-position error if you use the"
          " newest pose" % (speed, lat * 1000, speed * lat))
print("  At 640x400 this camera runs ~100 fps, so frame interval is 10 ms, not 33 ms. The win is")
print("  real but only if the pose is looked up at the CAPTURE timestamp.")

# ---------------------------------------------------------------------------------------------
print()
print("=" * 78)
print("%d passed, %d failed" % (PASS, FAIL))
print("=" * 78)
print()
if FAIL:
    print("Geometry is WRONG. Do not build on it until these pass.")
    sys.exit(1)
print("Geometry verified. Absolute distances above are provisional until the camera is calibrated;")
print("the transform chain itself is confirmed correct.")
