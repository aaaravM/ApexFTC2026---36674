#!/usr/bin/env python3
"""
Monochrome ball detector for BIOBUZZ.  Finds balls in a GRAYSCALE image, with no colour at all.

================================================================================================
WHY THIS IS SHAPED THE WAY IT IS
================================================================================================

With a colour camera, "find the orange pixels" does most of the work and the geometry is an
afterthought.  With a mono camera that whole first stage is gone, so the burden shifts: finding
round blobs is easy and finding round blobs THAT ARE ACTUALLY BALLS is the hard part.  Anything
circular and mid-grey is a candidate -- another robot's compliant wheels and mecanum rollers, bolt
heads, ceiling lights, the AprilTag mounts.

So this detector spends very little effort on thresholding and a lot on GEOMETRIC PLAUSIBILITY.
Three filters do the real work, and all three are nearly free:

  1. HORIZON.  A ball resting on the tiles must appear below the horizon line.  A ray through a
     pixel above the horizon never meets the floor.  This discards everything on the field wall,
     on other robots' superstructure, and in the lighting rig.

  2. RADIUS CONSISTENCY.  This is the strong one.  Get range from the floor plane -- which does not
     need to know the ball's size -- then ask what radius a real ball WOULD subtend at that range.
     A blob whose measured radius matches neither POLLEN nor NECTAR is not a ball.  A 4 in robot
     wheel 30 in away projects to a radius no game ball could have at the range the floor says it
     is at, so it is rejected on arithmetic rather than on appearance.

  3. METHOD AGREEMENT.  Range from the floor plane and range from apparent size should agree for a
     ball on the ground.  Disagreement means it is airborne (the floor assumption is false, and the
     apparent-size answer is the right one) or it is not a ball.  Either is worth knowing.

Filter 2 is what makes a monochrome camera viable here.  It recovers most of the discrimination
colour would have given, using only the known physical sizes of the game pieces.

================================================================================================
THRESHOLDING, AND WHY BOTH POLARITIES
================================================================================================

A ball is not reliably brighter than the floor, nor reliably darker.  It depends on the venue
lighting, the ball's finish, and where the ball is relative to the lights.  A detector that only
looks for bright blobs works at home and fails at a competition.

So the default pipeline thresholds BOTH WAYS -- bright-relative-to-local-background and
dark-relative-to-local-background -- and takes the union.  Adaptive thresholding (rather than one
global cut) handles the brightness gradient that a wide lens always has across the floor.

A shaded sphere also has a bright side and a dark limb, which means a single global threshold tends
to slice a ball in half and destroy its circularity.  CLOSE morphology repairs that; without it the
circularity test rejects real balls.

================================================================================================
COST
================================================================================================

Everything here is O(pixels) plus contour work: blur, two adaptive thresholds, two morphology
passes, findContours.  No Hough transform in the default path -- HoughCircles is both slower and
notoriously parameter-fragile, and once the radius-consistency filter exists we do not need its
circle-fitting.  A Hough path is provided for comparison, not for the robot.
"""

import math

import cv2
import numpy as np

from ball_geometry import (
    ground_plane_position, predicted_radius_px, apparent_size_ground_range,
    MIN_TYPE_SEPARATION_PX,
)

POLLEN_D = 2.8
NECTAR_D = 3.6
DEFAULT_CANDIDATES = (("POLLEN", POLLEN_D), ("NECTAR", NECTAR_D))


class DetectParams:
    """Tunables, all in one place so the Java port has an obvious one-to-one mapping.

    blur_ksize        : Gaussian blur before thresholding. Odd. 0 disables.
    adaptive_block    : adaptive-threshold neighbourhood, odd, in pixels. Should be comfortably
                        larger than a ball's diameter in pixels at the FAR end of your range, or
                        the ball becomes its own background and vanishes.
    adaptive_c        : how far from the local mean counts as foreground. Raise to reject noise.
    morph_ksize       : structuring element for OPEN/CLOSE.
    min_radius_px     : below this, a blob carries too few pixels for a usable radius.
    max_radius_px     : above this it is something closer than any ball should be.
    min_circularity   : contourArea / (pi r^2). A disc is 1.0, a real shaded ball 0.6-0.9, a strip
                        of tape about 0.2.
    min_fill          : contourArea / boundingRect area. Cheap second shape test.
    radius_mode       : "equivalent" uses sqrt(area/pi); "enclosing" uses minEnclosingCircle.
                        Equivalent is the default and it matters -- see the CONTACT SHADOW note.
    shadow_bias_frac  : fraction subtracted from the measured radius to undo the contact-shadow
                        and morphology inflation. MUST BE RE-MEASURED ON REAL FOOTAGE; the default
                        came from synthetic scenes and is a placeholder, not a measurement.
    radius_tol_frac   : proportional part of the accept/reject gate.
    radius_tol_px     : CONSTANT part of the gate, in pixels. Both are needed.

                        A purely fractional tolerance is the wrong shape. Radius measurement noise
                        is roughly constant in PIXELS, so a fixed fraction is far too tight at long
                        range and needlessly loose up close: +/-1 px is 3% of a 30 px near ball but
                        20% of a 5 px far one. A fraction-only gate therefore throws away distant
                        balls while waving through nonsense nearby.

                        Measured on synthetic frames, the radius inflation from the contact shadow
                        is NOT a constant fraction either. The ratio of measured to true radius runs
                        about 1.20 at 20 in, rises to 1.38 around 56-84 in, then COLLAPSES to 1.03
                        past 100 in -- because the shadow crescent stops being thick enough to
                        survive morphological opening and simply drops out of the blob. No single
                        bias value can track that, which is the real reason the gate needs a pixel
                        term: it has to absorb a modelling error that genuinely varies with range.
    max_range_in      : beyond this, ignore detections. Rows just below the horizon give absurd
                        ranges, so this gate is required, not optional.
    consistency_max   : reject when the two range methods disagree by more than this factor.
    airborne_min      : above this ratio, call it airborne rather than rejecting it.
    """

    def __init__(self, **kw):
        self.blur_ksize = 5
        self.adaptive_block = 51
        self.adaptive_c = 7
        self.morph_ksize = 5
        self.min_radius_px = 4
        self.max_radius_px = 120
        self.min_circularity = 0.55
        self.min_fill = 0.50
        self.radius_mode = "equivalent"
        self.shadow_bias_frac = 0.20
        self.radius_tol_frac = 0.22
        self.radius_tol_px = 2.0
        self.max_range_in = 160.0
        self.consistency_max = 1.35
        self.airborne_min = 1.35
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("unknown DetectParams field: %s" % k)
            setattr(self, k, v)


class Detection:
    """One accepted ball."""

    def __init__(self, u, v, radius_px, ball_type, diameter_in, x_in, y_in, range_in,
                 predicted_radius_px_, separation_px, circularity, consistency, airborne):
        self.u, self.v = u, v
        self.radius_px = radius_px
        self.type = ball_type
        self.diameter_in = diameter_in
        self.x_in, self.y_in = x_in, y_in
        self.range_in = range_in
        self.predicted_radius_px = predicted_radius_px_
        self.separation_px = separation_px
        self.circularity = circularity
        self.consistency = consistency
        self.airborne = airborne

    @property
    def bearing_rad(self):
        """Bearing to the ball in the robot frame, radians, CCW positive."""
        return math.atan2(self.y_in, self.x_in)

    @property
    def type_trustworthy(self):
        """False when the two ball types are too similar in apparent size to tell apart here."""
        return self.separation_px >= MIN_TYPE_SEPARATION_PX

    def __repr__(self):
        return ("<%s %s r=%.1fpx at (%.1f, %.1f) range %.1f in bearing %+.1f deg%s>"
                % (self.type, "" if self.type_trustworthy else "(uncertain)", self.radius_px,
                   self.x_in, self.y_in, self.range_in, math.degrees(self.bearing_rad),
                   " AIRBORNE" if self.airborne else ""))


def build_mask(gray, p):
    """Grayscale -> binary foreground, catching both bright and dark balls.

    Returns (mask, mask_bright, mask_dark) so a tuner can show which polarity fired.
    """
    img = gray
    if p.blur_ksize and p.blur_ksize >= 3:
        k = p.blur_ksize | 1
        img = cv2.GaussianBlur(img, (k, k), 0)

    block = max(3, p.adaptive_block | 1)

    # Bright-relative-to-local-background, then dark. The union covers both lighting regimes, so
    # the detector does not silently depend on which one the venue happens to give us.
    bright = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, block, -abs(p.adaptive_c))
    dark = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, block, abs(p.adaptive_c))
    mask = cv2.bitwise_or(bright, dark)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.morph_ksize, p.morph_ksize))
    # OPEN first to drop speckle, then CLOSE to heal the dark limb and the specular highlight that
    # otherwise split one ball into fragments and destroy its circularity.
    #
    # ONE closing iteration, not two. Closing is what welds the contact shadow onto the ball, so
    # every extra iteration costs radius accuracy. One is enough to heal a split ball; two measured
    # noticeably worse on synthetic scenes.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
    return mask, bright, dark


def find_candidates(mask, p):
    """Binary mask -> round-enough blobs. Returns [(u, v, radius_px, circularity, fill)].

    THE CONTACT SHADOW PROBLEM, and why radius is measured the way it is
        A ball resting on the tiles casts a shadow at its contact point.  That shadow is DARKER
        than the floor, so the dark-polarity threshold claims it as foreground, and morphological
        closing then welds it onto the ball.  The blob you get is the ball plus a crescent hanging
        below it.

        That inflates the measured radius, and the consequences are worse than they sound.  Radius
        is what tells POLLEN from NECTAR and what gates out non-balls, so a biased radius makes the
        detector both misclassify real balls AND reject them for "matching no ball at this range".
        On synthetic scenes an uncorrected minEnclosingCircle came out about 45% large, which was
        enough to lose two balls out of five and to call every POLLEN a NECTAR.

        Two mitigations, both here:

        1. EQUIVALENT-AREA RADIUS, sqrt(area/pi), instead of minEnclosingCircle.  The enclosing
           circle is the worst possible choice: it is set by the single furthest point, so the
           shadow crescent drags it out directly.  Area grows far less, because the crescent is
           thin.  Equivalent radius also degrades gracefully when a ball is clipped at the frame
           edge, where the enclosing circle goes badly wrong.

        2. CENTROID from image moments, not the enclosing-circle centre.  The shadow sits below the
           ball, so it pulls the enclosing-circle centre downward, and a lower pixel row reads as
           nearer on the floor plane.  The area centroid moves much less.

        A residual bias remains, and shadow_bias_frac removes it.  THAT CONSTANT IS A PLACEHOLDER
        measured off synthetic renders -- it depends on real lighting, the real floor and the real
        ball finish, so it must be re-measured from footage.  The way to measure it: put a ball at
        a tape-measured distance, read the detector's radius, compare against
        predicted_radius_px() for the true range, and set the fraction to the difference.
    """
    out = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        (eu, ev), er = cv2.minEnclosingCircle(c)
        area = cv2.contourArea(c)
        if area <= 0 or er <= 0:
            continue

        # Circularity always uses the enclosing circle -- that is the shape test, and the shadow
        # legitimately makes a ball-plus-shadow blob less circular.
        circ = area / (math.pi * er * er)
        if circ < p.min_circularity:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        fill = area / float(bw * bh) if bw * bh > 0 else 0.0
        if fill < p.min_fill:
            continue

        if p.radius_mode == "equivalent":
            r = math.sqrt(area / math.pi)
        else:
            r = er
        r *= (1.0 - p.shadow_bias_frac)
        if r < p.min_radius_px or r > p.max_radius_px:
            continue

        m = cv2.moments(c)
        if m["m00"] > 0:
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
        else:
            u, v = eu, ev

        out.append((u, v, r, circ, fill))
    return out


def classify_candidate(u, v, radius_px, circ, intr, pose, p, candidates=DEFAULT_CANDIDATES):
    """Apply the three geometric filters. Returns a Detection, or None with a reason.

    Returns (detection_or_None, reason_string). The reason is what makes this debuggable: when the
    robot "cannot see a ball that is right there", the reason tells you which filter ate it rather
    than leaving you guessing.
    """
    scored = []
    for name, D in candidates:
        gp = ground_plane_position(u, v, intr, pose, D)
        if gp is None:
            return None, "above horizon"
        bx, by, rng = gp
        if rng <= 0 or rng > p.max_range_in:
            return None, "range %.0f in beyond max %.0f" % (rng, p.max_range_in)
        pred = predicted_radius_px(rng, intr, pose, D)
        if pred is None or pred <= 0:
            return None, "no predicted radius"
        scored.append((name, D, bx, by, rng, pred,
                       abs(pred - radius_px) / pred,     # relative, for the gate
                       abs(pred - radius_px)))           # absolute, for choosing the type

    if not scored:
        return None, "no candidate type"

    # Choose on ABSOLUTE pixel difference. Relative error would systematically favour the larger
    # candidate -- the same absolute miss is a smaller fraction of a bigger radius -- which stacks
    # on top of the contact-shadow bias and makes everything a NECTAR. The GATE below stays
    # relative, which is right for "is this plausibly a ball at all".
    scored.sort(key=lambda s: s[7])
    name, D, bx, by, rng, pred, relerr, _abserr = scored[0]

    # Separation is how far the chosen prediction sits from the NEAREST other prediction: a
    # property of the geometry, not of this measurement. Below ~1.5 px the types are physically
    # indistinguishable here however good the detector is.
    others = [s[5] for s in scored[1:]]
    separation = min((abs(pred - o) for o in others), default=float("inf"))

    # FILTER 2: the radius must be plausible for SOME real ball at the range the floor implies.
    abserr = abs(pred - radius_px)
    allowed = p.radius_tol_px + p.radius_tol_frac * pred
    if abserr > allowed:
        return None, ("radius %.1f px matches no ball at %.0f in (best %s predicts %.1f px, off "
                      "by %.1f px, allowed %.1f)" % (radius_px, rng, name, pred, abserr, allowed))

    # FILTER 3: do the two independent range methods agree?
    sz = apparent_size_ground_range(radius_px, intr, pose, D)
    consistency = (sz / rng) if (sz and rng > 0) else None
    airborne = False
    if consistency is not None:
        if consistency >= p.airborne_min:
            airborne = True            # further than the floor model says -> not on the floor
        elif consistency > p.consistency_max or consistency < 1.0 / p.consistency_max:
            return None, "range methods disagree (floor %.0f in vs size %.0f in)" % (rng, sz)

    return Detection(u, v, radius_px, name, D, bx, by, rng, pred, separation, circ,
                     consistency, airborne), "ok"


def detect(gray, intr, pose, p=None, candidates=DEFAULT_CANDIDATES, want_rejects=False):
    """Full pipeline: grayscale image -> list of Detection, nearest first.

    gray : single-channel uint8. A 3-channel mono frame must be converted by the caller, so that
           the channel choice is explicit rather than accidental.
    Returns (detections, mask, rejects) where rejects is [(u, v, r, reason)] when want_rejects.
    """
    if p is None:
        p = DetectParams()
    if gray.ndim != 2:
        raise ValueError("detect() wants a single-channel image; convert explicitly first")

    mask, _, _ = build_mask(gray, p)
    dets, rejects = [], []
    for (u, v, r, circ, _fill) in find_candidates(mask, p):
        d, reason = classify_candidate(u, v, r, circ, intr, pose, p, candidates)
        if d is not None:
            dets.append(d)
        elif want_rejects:
            rejects.append((u, v, r, reason))

    dets.sort(key=lambda d: d.range_in)
    return dets, mask, rejects


# =============================================================================================
# Synthetic scene rendering -- so the detector can be tested without a camera or a field.
# =============================================================================================

def render_scene(intr, pose, balls, distractors=(), floor_grey=110, light=(-0.4, -0.6, 0.7),
                 noise_sigma=3.0, vignette=0.25, seed=0):
    """Render a plausible grayscale view of balls on a floor, with known ground truth.

    balls       : [(x_in, y_in, diameter_in)] in the ROBOT frame
    distractors : [(u, v, radius_px, grey)] drawn directly in image space -- things that are round
                  and the wrong size, or round and above the horizon. The detector SHOULD reject
                  these, and the test asserts that it does.

    The balls are rendered with Lambertian shading rather than as flat discs, deliberately: a
    shaded sphere has a bright side and a dark limb, which is exactly what breaks naive
    thresholding and splits blobs. Testing against flat discs would flatter the detector and prove
    nothing about the real thing.
    """
    rng = np.random.default_rng(seed)
    h, w = intr.height, intr.width

    # Floor with a gradient and a vignette, because a wide lens always gives you both.
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    img = np.full((h, w), float(floor_grey), dtype=np.float32)
    img += 18.0 * (yy / h) - 9.0                                  # nearer floor is brighter
    rr = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
    img *= (1.0 - vignette * np.clip(rr, 0, 1.4) ** 2)

    lx, ly, lz = light
    ln = math.sqrt(lx * lx + ly * ly + lz * lz)
    lx, ly, lz = lx / ln, ly / ln, lz / ln

    truth = []
    for (bx, by, D) in balls:
        # Project the ball centre the same way test_geometry.project does.
        lens_x, lens_y = pose.lens_in_robot_frame()
        wx, wy, wz = bx - lens_x, by - lens_y, D / 2.0 - pose.height_in
        cyaw, syaw = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        w1x = wx * cyaw + wy * syaw
        w1y = -wx * syaw + wy * cyaw
        sp, cp = math.sin(pose.pitch_rad), math.cos(pose.pitch_rad)
        vx = -w1y
        vy = -w1x * sp - wz * cp
        vz = w1x * cp - wz * sp
        if vz <= 1e-6:
            continue
        u = intr.cx + intr.fx * vx / vz
        v = intr.cy + intr.fy * vy / vz
        L = math.sqrt(vx * vx + vy * vy + vz * vz)
        r = intr.fx * (D / 2.0) / L
        if r < 1.0:
            continue

        # Lambertian sphere: surface normal from the distance off centre, shaded by the light.
        y0, y1 = max(0, int(v - r - 2)), min(h, int(v + r + 3))
        x0, x1 = max(0, int(u - r - 2)), min(w, int(u + r + 3))
        if y1 <= y0 or x1 <= x0:
            continue
        sy, sx = np.mgrid[y0:y1, x0:x1].astype(np.float32)
        dx, dy = (sx - u) / r, (sy - v) / r
        d2 = dx * dx + dy * dy
        inside = d2 <= 1.0
        nz = np.sqrt(np.clip(1.0 - d2, 0.0, 1.0))
        lam = np.clip(dx * lx + dy * ly + nz * lz, 0.0, 1.0)
        shade = 55.0 + 165.0 * lam                     # ambient + diffuse
        patch = img[y0:y1, x0:x1]
        patch[inside] = shade[inside]

        # Contact shadow: a real ball on a floor darkens the tiles just below it, which adds a
        # non-circular dark region right at the blob's edge. Good: it makes the test honest.
        sh = (d2 <= 1.45) & (~inside) & (dy > 0.15)
        patch[sh] *= 0.78
        img[y0:y1, x0:x1] = patch

        truth.append({"x_in": bx, "y_in": by, "diameter_in": D, "u": u, "v": v, "radius_px": r,
                      "range_in": math.hypot(bx, by)})

    for (du, dv, dr, dgrey) in distractors:
        cv2.circle(img, (int(du), int(dv)), int(dr), float(dgrey), -1)

    img += rng.normal(0.0, noise_sigma, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8), truth


def draw_overlay(gray, dets, rejects=()):
    """Annotate a frame for human inspection. Returns a BGR image."""
    out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for (u, v, r, reason) in rejects:
        cv2.circle(out, (int(u), int(v)), int(r), (60, 60, 160), 1)
        cv2.putText(out, reason[:30], (int(u) - int(r), int(v) + int(r) + 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (80, 80, 200), 1)
    for i, d in enumerate(dets):
        colour = (0, 255, 0) if i == 0 else (0, 200, 255)
        if d.airborne:
            colour = (255, 180, 0)
        cv2.circle(out, (int(d.u), int(d.v)), int(d.radius_px), colour, 2)
        cv2.drawMarker(out, (int(d.u), int(d.v)), (0, 0, 255), cv2.MARKER_CROSS, 7, 1)
        label = "%s %.0fin %+.0fdeg" % (d.type[:3], d.range_in, math.degrees(d.bearing_rad))
        if not d.type_trustworthy:
            label += " ?"
        cv2.putText(out, label, (int(d.u) - int(d.radius_px), int(d.v) - int(d.radius_px) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, colour, 1)
    return out
