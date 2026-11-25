#!/usr/bin/env python3
"""
Pixel <-> field geometry for mono ball detection.  Pure maths, no OpenCV, no camera.

This module is deliberately separate from the detector so it can be TESTED against known ground
truth (see test_geometry.py) and so the Java port on the robot can mirror it line for line.

================================================================================================
THE KEY IDEA: USE THE FLOOR, NOT THE BALL'S APPARENT SIZE
================================================================================================

There are two ways to get range to a ball from one camera.

  (1) APPARENT SIZE.  A ball of known diameter D subtending radius r pixels is at slant distance
          L = fx * (D/2) / r
      This is what the old colour BallTracker used.  It needs to know WHICH ball it is, because D
      differs between POLLEN and NECTAR -- and in monochrome we cannot tell them apart by colour.
      So it is circular: we need the size to get the range, and the range to judge the size.

  (2) THE GROUND PLANE.  A ball resting on the floor has its centre at a known height, D/2, above
      the tiles.  Back-project the pixel into a ray and intersect that ray with the horizontal
      plane z = D/2.  Range falls out of the geometry.

Method (2) is better here for three separate reasons:

  a) ACCURACY.  Differentiate both models.  For method (1), dL/dr = -L^2 / (fx * D/2), so the range
     error per pixel of radius error is L^2 / (fx * D/2).  For method (2), the error per pixel of
     ROW error is about L^2 / (fy * H), where H is the camera height above the ball centre.
     The ratio of the two is H / (D/2).  With a camera 14 in up and a 2.8 in POLLEN, that is
     12.6 / 1.4 = 9x better for the ground-plane method.

  b) MEASUREMENT NOISE.  A blob's CENTROID is an average over hundreds of pixels and is stable to a
     fraction of a pixel.  Its RADIUS depends on exactly where the threshold cuts the soft edge of
     the ball, which moves with lighting and with range.  So method (1)'s input is the noisier of
     the two, and the real-world gap is wider than the 9x above.

  c) IT BREAKS THE CIRCULARITY.  The plane z = D/2 depends only weakly on D -- swapping a 2.8 in
     ball for a 3.6 in one moves the plane by 0.4 in, against a camera height of ~14 in, a 3%
     change in range.  So we can get range WITHOUT knowing which ball it is, and then use the
     measured radius to decide which it is.  See classify_by_radius() below.

  Method (1) still earns its place: it is the only option for a ball in FLIGHT (which is not on the
  floor, so the plane assumption is false), and comparing the two gives a free consistency check.

================================================================================================
THE FREE FALSE-POSITIVE FILTER
================================================================================================

Without colour, anything round and mid-grey is a ball candidate: another robot's compliant wheels,
mecanum rollers, bolt heads, lights.  The ground-plane model hands us a strong filter for nothing:

  A ball on the floor MUST appear BELOW THE HORIZON.

A ray through a pixel above the horizon travels upward forever and never meets the floor, so
ground_plane_range() returns None for it.  That one test discards every candidate up on the field
wall, on another robot's superstructure, or in the ceiling lights.  Use it before anything
expensive.

And a second filter, nearly free: once we have range from the floor, we know what radius a real ball
WOULD subtend.  A blob whose measured radius matches neither ball type at that range is not a ball.

================================================================================================
FRAME CONVENTIONS -- these match the rest of this codebase
================================================================================================

ROBOT/FIELD frame:  +X forward, +Y left, +Z up.  Inches and radians.  Heading CCW positive.
CAMERA frame (OpenCV):  +x right, +y DOWN, +z forward along the optical axis.

Camera pose is given as height above the floor, pitch (positive = tilted DOWN toward the floor),
and an offset from the robot's tracking point.
"""

import math


class CameraPose:
    """Where the lens is and where it points.

    height_in : lens centre above the tiles, inches
    pitch_deg : positive = tilted DOWN toward the floor. A camera aimed at the horizon is 0.
    x_in, y_in: lens offset from the robot's tracking point, robot frame (+X fwd, +Y left)
    yaw_rad   : lens direction relative to robot forward. For a TURRET-MOUNTED camera this is the
                turret angle, and the lens also ORBITS the pivot -- see lens_in_robot_frame().
    pivot_x_in, pivot_y_in : turret rotation axis in the robot frame. Only used when on_turret.
    on_turret : if True, x_in/y_in are the lens offset FROM THE PIVOT with the turret at zero.
    """

    def __init__(self, height_in, pitch_deg, x_in=0.0, y_in=0.0,
                 yaw_rad=0.0, pivot_x_in=0.0, pivot_y_in=0.0, on_turret=False):
        self.height_in = float(height_in)
        self.pitch_rad = math.radians(pitch_deg)
        self.x_in = float(x_in)
        self.y_in = float(y_in)
        self.yaw_rad = float(yaw_rad)
        self.pivot_x_in = float(pivot_x_in)
        self.pivot_y_in = float(pivot_y_in)
        self.on_turret = bool(on_turret)

    def lens_in_robot_frame(self):
        """Lens position in the robot frame, accounting for a turret-mounted lens orbiting its pivot.

        For a chassis-mounted camera this is just (x_in, y_in).  For a turret-mounted one the lens
        swings around the pivot as the turret rotates, so the offset must be ROTATED by the turret
        angle before being added to the pivot.  Forgetting this is a real error: a lens 4 in ahead
        of the pivot moves 8 in between turret +90 and -90.
        """
        if not self.on_turret:
            return self.x_in, self.y_in
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        lx = self.pivot_x_in + (self.x_in * c - self.y_in * s)
        ly = self.pivot_y_in + (self.x_in * s + self.y_in * c)
        return lx, ly


class Intrinsics:
    """Pinhole intrinsics at a specific capture resolution.

    ALWAYS carry the resolution with the numbers.  Focal length scales linearly with capture width,
    so fx measured at 1280 is exactly 2x the fx that applies at 640, and using the wrong one is a
    silent 2x error in every range.  scaled_to() exists so that mistake is hard to make.
    """

    def __init__(self, fx, fy, cx, cy, width, height):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.width, self.height = int(width), int(height)

    def scaled_to(self, width, height):
        """Same lens, different capture size.

        WARNING: this is only valid when the sensor SCALES (bins or resamples the full field of
        view).  If it CROPS instead -- which is what changing aspect ratio often does, e.g. 1280x800
        16:10 down to 640x480 4:3 -- then the field of view changes and the principal point moves,
        and this scaling is wrong.  Calibrate at each aspect ratio you intend to use.
        """
        sx = width / float(self.width)
        sy = height / float(self.height)
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, width, height)

    def hfov_deg(self):
        return 2.0 * math.degrees(math.atan(self.width / (2.0 * self.fx)))

    def vfov_deg(self):
        return 2.0 * math.degrees(math.atan(self.height / (2.0 * self.fy)))


def pixel_ray_camera(u, v, intr):
    """Unit-ish direction through pixel (u,v) in the CAMERA frame (+x right, +y down, +z fwd).

    Not normalised; z is 1 by construction, which is all the callers need.
    """
    return ((u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0)


def _ray_camera_to_world(a, b, pitch_rad):
    """Rotate a camera-frame ray (a, b, 1) into the world frame (+X fwd, +Y left, +Z up).

    The camera basis expressed in world coordinates, for a lens pitched DOWN by `pitch`:
        x_cam (right)   -> ( 0,        -1,  0       )
        y_cam (down)    -> (-sin p,     0, -cos p   )
        z_cam (forward) -> ( cos p,     0, -sin p   )
    (Right-handed: x_cam x y_cam = z_cam, which test_geometry.py asserts.)

    Yaw is applied by the caller, after this, so that a turret-mounted camera can share the code.
    """
    sp, cp = math.sin(pitch_rad), math.cos(pitch_rad)
    rx = cp - b * sp
    ry = -a
    rz = -sp - b * cp
    return rx, ry, rz


def ground_plane_position(u, v, intr, pose, ball_diameter_in):
    """Where a FLOOR-RESTING ball at pixel (u,v) is, in the ROBOT frame. The primary range method.

    Returns (x_in, y_in, range_in) in the robot frame, or None if the ray never meets the floor
    plane -- which means the pixel is at or above the horizon, so this cannot be a ball on the
    ground. That None is a feature: use it as a false-positive filter.
    """
    a, b, _ = pixel_ray_camera(u, v, intr)
    rx, ry, rz = _ray_camera_to_world(a, b, pose.pitch_rad)

    # Rotate by the lens yaw (turret angle) into the robot frame.
    cy_, sy_ = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    wx = rx * cy_ - ry * sy_
    wy = rx * sy_ + ry * cy_

    # The ball's centre sits at z = D/2. The camera is at z = height.
    target_z = ball_diameter_in / 2.0
    dz = target_z - pose.height_in            # negative: the ball centre is below the lens
    if rz >= -1e-9:
        return None                           # ray is level or rising: never reaches the floor
    t = dz / rz                               # both negative -> t positive
    if t <= 0:
        return None

    lens_x, lens_y = pose.lens_in_robot_frame()
    bx = lens_x + t * wx
    by = lens_y + t * wy
    return bx, by, math.hypot(bx, by)


def apparent_size_slant_distance(radius_px, intr, ball_diameter_in):
    """Slant (3-D) distance from the lens to the ball centre, from its apparent radius.

        L = fx * (D/2) / r

    This is the small-angle form. The exact relation for a sphere is r = f*R/sqrt(L^2 - R^2); at
    L = 10*R the two differ by 0.5%, and we are always far past that, so the simple form is fine.

    Returns None for a non-positive radius rather than dividing by zero.
    """
    if radius_px <= 0:
        return None
    return intr.fx * (ball_diameter_in / 2.0) / radius_px


def apparent_size_ground_range(radius_px, intr, pose, ball_diameter_in):
    """Horizontal range from apparent size, for comparison with the ground-plane answer.

    Converts the slant distance to a horizontal one by removing the vertical leg. Returns None if
    the slant distance is shorter than the height difference, which is geometrically impossible and
    therefore indicates a bad radius measurement.
    """
    L = apparent_size_slant_distance(radius_px, intr, ball_diameter_in)
    if L is None:
        return None
    dz = pose.height_in - ball_diameter_in / 2.0
    if L <= abs(dz):
        return None
    return math.sqrt(L * L - dz * dz)


def predicted_radius_px(range_in, intr, pose, ball_diameter_in):
    """What radius a ball of this diameter WOULD subtend at this horizontal range.

    This is the inverse of apparent_size_ground_range, and it is how we classify ball type without
    colour: get range from the floor (which barely depends on D), then ask which ball's predicted
    radius matches what we measured.
    """
    dz = pose.height_in - ball_diameter_in / 2.0
    L = math.hypot(range_in, dz)
    if L <= 0:
        return None
    return intr.fx * (ball_diameter_in / 2.0) / L


def classify_by_radius(u, v, radius_px, intr, pose, candidates):
    """Decide WHICH ball this is, using size, with no colour information at all.

    candidates: list of (name, diameter_in), e.g. [("POLLEN", 2.8), ("NECTAR", 3.6)]

    Method, and why it works:
      1. Range from the ground plane. This barely depends on D (a 0.8 in diameter difference moves
         the assumed centre height by 0.4 in against a ~14 in camera height), so we can compute it
         before knowing the type -- which is what breaks the circular dependency.
      2. For each candidate, the radius it would subtend at that range.
      3. Pick the closest match.

    Returns a dict with the chosen type, the range, and -- importantly -- a `separation` figure:
    the gap between the two candidates' predicted radii in pixels. When that gap falls below about
    a pixel the two types are physically indistinguishable at this range, no matter how good the
    detector is. Report it rather than pretending to a confident answer; the caller should fall
    back on the intake colour sensor beyond that distance.

    Returns None if the pixel is above the horizon.
    """
    best = None
    preds = []
    for name, D in candidates:
        gp = ground_plane_position(u, v, intr, pose, D)
        if gp is None:
            continue
        _, _, rng = gp
        pred = predicted_radius_px(rng, intr, pose, D)
        if pred is None:
            continue
        preds.append((name, D, rng, pred, abs(pred - radius_px)))

    if not preds:
        return None

    preds.sort(key=lambda p: p[4])
    name, D, rng, pred, err = preds[0]

    separation = abs(preds[0][3] - preds[1][3]) if len(preds) > 1 else float("inf")
    gp = ground_plane_position(u, v, intr, pose, D)
    bx, by, _ = gp

    return {
        "type": name,
        "diameter_in": D,
        "x_in": bx,
        "y_in": by,
        "range_in": rng,
        "radius_px": radius_px,
        "predicted_radius_px": pred,
        "radius_error_px": err,
        "separation_px": separation,
        "type_trustworthy": separation >= 1.5,   # below this, size cannot separate the two
        "all": [{"type": p[0], "range_in": p[2], "predicted_radius_px": p[3]} for p in preds],
    }


def consistency_ratio(u, v, radius_px, intr, pose, ball_diameter_in):
    """Cross-check the two range methods. Returns (ground_range, size_range, ratio) or None.

    A real ball on the floor should give close agreement. Large disagreement means one of:
      - it is not a ball (a round thing of a different real size: a wheel, a light)
      - it is AIRBORNE, so the floor assumption is false -- in which case the apparent-size answer
        is the correct one and the ground-plane answer should be discarded
      - the radius is badly measured (blown-out highlight, two balls merged into one blob)

    So the ratio is useful both as a reject test and as an airborne-ball detector.
    """
    gp = ground_plane_position(u, v, intr, pose, ball_diameter_in)
    sz = apparent_size_ground_range(radius_px, intr, pose, ball_diameter_in)
    if gp is None or sz is None:
        return None
    rng = gp[2]
    if rng <= 0:
        return None
    return rng, sz, sz / rng


def robot_to_field(x_in, y_in, robot_x, robot_y, robot_heading_rad):
    """Robot-frame point -> field frame, given the robot pose AT THE FRAME'S CAPTURE TIME.

    Use the pose from the capture timestamp, not the newest one. At 50 in/s with 100 ms of vision
    latency the robot has moved 5 in since the shutter, and that error lands straight on the ball's
    field position.
    """
    c, s = math.cos(robot_heading_rad), math.sin(robot_heading_rad)
    return robot_x + x_in * c - y_in * s, robot_y + x_in * s + y_in * c


def range_error_per_pixel(range_in, intr, pose, ball_diameter_in):
    """How many inches of range error one pixel of measurement error causes, for both methods.

    Returns (ground_plane_in_per_px, apparent_size_in_per_px). Use it to pick an operating envelope
    and to set a sane maximum detection range, rather than guessing one.

    Derivations:
        ground plane:    d ~ H * fy / (v - v_horizon)   =>   |dd/dv| = d^2 / (fy * H)
        apparent size:   L = fx * R / r                 =>   |dL/dr| = L^2 / (fx * R)
    """
    H = pose.height_in - ball_diameter_in / 2.0
    R = ball_diameter_in / 2.0
    gp = range_in * range_in / (intr.fy * H) if H > 0 else float("inf")
    L = math.hypot(range_in, H)
    sz = L * L / (intr.fx * R)
    return gp, sz
