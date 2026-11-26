#!/usr/bin/env python3
"""
End-to-end test of the mono ball detector against synthetic scenes with known ground truth.

WHAT THIS PROVES, AND WHAT IT DOES NOT
    PROVES: the pipeline runs, the three geometric filters do what they claim, balls at known
    positions are recovered to a known accuracy, and round objects that are NOT balls are rejected
    for the stated reason.

    DOES NOT PROVE: that it works on the real field.  Synthetic shading is not real shading, and no
    renderer reproduces venue lighting, motion blur, scuffed tiles, or another robot parked in
    frame.  Treat the numbers here as a floor on performance, not a prediction.  The first real
    footage will move them, and that is expected.

    The renderer deliberately makes life hard: Lambertian shading so each ball has a bright side
    and a dark limb, a contact shadow that breaks circularity at the blob edge, a floor brightness
    gradient, a lens vignette, and sensor noise.  Testing against flat discs would prove nothing.

RUN
    python tools/vision/test_detect.py
    python tools/vision/test_detect.py --save    # also write annotated PNGs to captures/
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

from ball_geometry import CameraPose, Intrinsics
from ball_detect import (
    DetectParams, detect, render_scene, draw_overlay, POLLEN_D, NECTAR_D,
)

# PROVISIONAL intrinsics -- the Arducam is not calibrated yet. 70 deg HFOV at 640x400.
# Absolute inches below scale with fx; the pass/fail logic is self-consistent either way.
INTR = Intrinsics(fx=457.0, fy=457.0, cx=320.0, cy=200.0, width=640, height=400)
POSE = CameraPose(height_in=14.0, pitch_deg=18.0)

PASS, FAIL = 0, 0
SAVE = False
OUTDIR = "captures"


def check(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        print("  FAIL  %s   %s" % (name, detail))


def save(img, name):
    if not SAVE:
        return
    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.join(OUTDIR, name)
    cv2.imwrite(path, img)
    print("        wrote %s" % path)


def match(dets, truth, max_err_in):
    """Pair each truth ball with the nearest detection. Returns (pairs, missed, extra)."""
    unused = list(dets)
    pairs, missed = [], []
    for t in truth:
        best, bestd = None, 1e9
        for d in unused:
            e = math.hypot(d.x_in - t["x_in"], d.y_in - t["y_in"])
            if e < bestd:
                best, bestd = d, e
        if best is not None and bestd <= max_err_in:
            pairs.append((t, best, bestd))
            unused.remove(best)
        else:
            missed.append(t)
    return pairs, missed, unused


# ---------------------------------------------------------------------------------------------
def test_single_ball():
    print()
    print("=" * 78)
    print("1. One ball, straight ahead, at a range of known truth")
    print("=" * 78)
    for rng in (24.0, 36.0, 48.0, 72.0):
        img, truth = render_scene(INTR, POSE, [(rng, 0.0, POLLEN_D)], seed=1)
        dets, mask, rejects = detect(img, INTR, POSE, want_rejects=True)
        pairs, missed, extra = match(dets, truth, max_err_in=6.0)
        ok = len(pairs) == 1
        detail = ""
        if ok:
            t, d, e = pairs[0]
            detail = ("range truth %.1f got %.1f (err %+.2f in), type %s, r %.1f px (pred %.1f)"
                      % (t["range_in"], d.range_in, d.range_in - t["range_in"], d.type,
                         d.radius_px, d.predicted_radius_px))
        else:
            detail = "found %d dets, %d rejects: %s" % (
                len(dets), len(rejects), "; ".join(r[3] for r in rejects[:3]))
        check("POLLEN at %.0f in detected" % rng, ok, detail)
        if ok:
            print("        %s" % detail)
        save(draw_overlay(img, dets, rejects), "t1_pollen_%03din.png" % int(rng))


def test_many_balls():
    print()
    print("=" * 78)
    print("2. Several balls at once, both types, spread across the frame")
    print("=" * 78)
    balls = [
        (26.0, -8.0, POLLEN_D),
        (34.0, 10.0, NECTAR_D),
        (48.0, -2.0, POLLEN_D),
        (62.0, 16.0, NECTAR_D),
        (80.0, -14.0, POLLEN_D),
    ]
    img, truth = render_scene(INTR, POSE, balls, seed=2)
    dets, mask, rejects = detect(img, INTR, POSE, want_rejects=True)
    pairs, missed, extra = match(dets, truth, max_err_in=8.0)

    print("  rendered %d, in frame %d, detected %d, matched %d"
          % (len(balls), len(truth), len(dets), len(pairs)))
    for t, d, e in pairs:
        print("    truth (%6.1f,%6.1f) %s -> got (%6.1f,%6.1f) %s  pos err %.2f in"
              % (t["x_in"], t["y_in"], "POLLEN" if t["diameter_in"] == POLLEN_D else "NECTAR",
                 d.x_in, d.y_in, d.type, e))
    for t in missed:
        print("    MISSED (%.1f,%.1f) r=%.1f px at range %.0f"
              % (t["x_in"], t["y_in"], t["radius_px"], t["range_in"]))
    for r in rejects:
        print("    rejected blob r=%.1f px: %s" % (r[2], r[3]))

    check("at least 4 of %d in-frame balls found" % len(truth), len(pairs) >= min(4, len(truth)),
          "matched %d" % len(pairs))
    check("no spurious extra detections", len(extra) == 0,
          "%d extra: %s" % (len(extra), extra))
    save(draw_overlay(img, dets, rejects), "t2_many.png")


def test_distractor_rejection():
    print()
    print("=" * 78)
    print("3. Round things that are NOT balls must be rejected")
    print("=" * 78)

    # Horizon row for this pose, so distractors can be placed deliberately above or below it.
    horizon_v = INTR.cy + INTR.fy * (-math.tan(POSE.pitch_rad))
    print("  horizon is at image row %.0f of %d" % (horizon_v, INTR.height))

    cases = [
        # (label, distractor (u, v, r, grey), should_be_rejected)
        ("ceiling light above horizon", (120, max(4, int(horizon_v) - 45), 22, 245), True),
        ("bright disc well above horizon", (520, max(4, int(horizon_v) - 70), 14, 250), True),
        # 40 px high up the frame: that row implies ~59 in, where a NECTAR subtends ~14 px.
        ("robot wheel, far too big for its range", (300, 150, 40, 70), True),
        ("bolt head, far too small for its range", (260, 360, 5, 40), True),
    ]

    for label, dist, should_reject in cases:
        img, truth = render_scene(INTR, POSE, [], distractors=[dist], seed=3)
        dets, mask, rejects = detect(img, INTR, POSE, want_rejects=True)
        rejected = len(dets) == 0
        reasons = "; ".join(r[3] for r in rejects) or "(blob not even found)"
        check("%s rejected" % label, rejected == should_reject,
              "got %d detections: %s" % (len(dets), [str(d) for d in dets]))
        if rejected:
            print("        reason: %s" % reasons[:110])
        save(draw_overlay(img, dets, rejects), "t3_%s.png" % label.split(",")[0].replace(" ", "_"))

    # And the important case: a real ball plus distractors together. The ball must survive.
    img, truth = render_scene(
        INTR, POSE, [(40.0, 0.0, POLLEN_D)],
        distractors=[(120, max(4, int(horizon_v) - 45), 22, 245), (520, 150, 40, 70)], seed=4)
    dets, mask, rejects = detect(img, INTR, POSE, want_rejects=True)
    pairs, missed, extra = match(dets, truth, max_err_in=8.0)
    check("real ball still found with distractors present", len(pairs) == 1,
          "found %d dets" % len(dets))
    check("distractors did not become detections", len(extra) == 0, "%d extra" % len(extra))
    save(draw_overlay(img, dets, rejects), "t3_ball_plus_distractors.png")


def test_known_limitation():
    """Document, and assert, what geometry alone CANNOT reject.

    The radius-consistency filter rejects a round object whose apparent size is wrong for the range
    its image position implies.  It therefore CANNOT reject a round object whose apparent size is
    RIGHT.  A 3.5 in compliant wheel sitting on the floor 18 in away is, in a monochrome image,
    geometrically indistinguishable from a 3.6 in NECTAR at 18.5 in -- same shape, same size, same
    place, no colour to separate them.  No amount of better code fixes that; the information is not
    in the image.

    This matters more than it might sound, because FTC wheel sizes are uncomfortably close to the
    game pieces: 3 in and 3.5 in and 4 in compliant wheels bracket POLLEN at 2.8 in and NECTAR at
    3.6 in almost exactly.

    This test asserts the detector DOES accept it, so that the limitation is pinned down by a test
    rather than discovered at a competition.  If a future change makes this pass by rejecting it,
    that change is either genuinely cleverer or is rejecting real balls -- and either way someone
    should look.

    What actually mitigates it, none of which is single-frame geometry:
      - TEMPORAL CONSISTENCY. A wheel is attached to a robot: it translates with the robot and is
        never still relative to its chassis. A ball on the floor is stationary in FIELD
        coordinates. Track in field coordinates and the two separate quickly.
      - CONTEXT. Wheels come in pairs or fours at a fixed spacing, at a fixed height, below a large
        occluding body. A learned classifier on a cropped patch gets this almost for free, which is
        the strongest argument in this project for a small trained model.
      - KEEP-OUT REGIONS. The robot knows from odometry where it is; much of the field cannot
        contain a loose ball.
      - DO NOT CARE. Driving at a wheel-shaped thing and failing to intake it costs a couple of
        seconds. Being wrong here is cheap, which is worth remembering before over-engineering it.
    """
    print()
    print("=" * 78)
    print("3b. KNOWN LIMITATION: a wheel-sized round thing in a ball-consistent place")
    print("=" * 78)
    # 37 px at row 330 is what a NECTAR would subtend at the ~18 in that row implies.
    img, _ = render_scene(INTR, POSE, [], distractors=[(300, 330, 37, 70)], seed=11)
    dets, _, rejects = detect(img, INTR, POSE, want_rejects=True)
    check("accepted as a ball, as expected -- geometry cannot separate these", len(dets) == 1,
          "got %d detections, %d rejects" % (len(dets), len(rejects)))
    if dets:
        print("        %s" % dets[0])
    print("        Mitigation is temporal tracking in FIELD coordinates, not a better single-frame")
    print("        filter: a wheel moves with its robot, a loose ball does not.")
    save(draw_overlay(img, dets, rejects), "t3b_known_limitation.png")


def test_range_accuracy():
    print()
    print("=" * 78)
    print("4. Range accuracy vs truth, and the usable envelope")
    print("=" * 78)
    print("  %-10s %-10s %-10s %-10s %s" % ("truth in", "got in", "err in", "err %", "type"))
    errs = []
    furthest = 0.0
    for rng in (20, 28, 36, 44, 56, 68, 84, 100, 120, 140):
        img, truth = render_scene(INTR, POSE, [(float(rng), 0.0, POLLEN_D)], seed=5 + rng)
        dets, _, _ = detect(img, INTR, POSE, want_rejects=False)
        pairs, missed, _ = match(dets, truth, max_err_in=max(8.0, 0.25 * rng))
        if not pairs:
            print("  %-10d %-10s %-10s %-10s %s" % (rng, "-", "-", "-", "NOT DETECTED"))
            continue
        t, d, _ = pairs[0]
        err = d.range_in - t["range_in"]
        errs.append(abs(err) / t["range_in"])
        furthest = max(furthest, rng)
        print("  %-10d %-10.1f %+-10.2f %+-10.1f %s"
              % (rng, d.range_in, err, 100 * err / t["range_in"],
                 d.type + ("" if d.type_trustworthy else " (uncertain)")))

    check("detected out to at least 100 in", furthest >= 100, "furthest %.0f in" % furthest)
    if errs:
        check("median range error under 10%%", float(np.median(errs)) < 0.10,
              "median %.1f%%" % (100 * np.median(errs)))
        print()
        print("  median %.1f%%, worst %.1f%% over %d detections"
              % (100 * np.median(errs), 100 * max(errs), len(errs)))


def test_type_classification():
    print()
    print("=" * 78)
    print("5. Type classification without colour")
    print("=" * 78)
    right = wrong = uncertain = 0
    for D, name in ((POLLEN_D, "POLLEN"), (NECTAR_D, "NECTAR")):
        for rng in (24, 32, 40, 52, 64, 80, 100):
            img, truth = render_scene(INTR, POSE, [(float(rng), 0.0, D)], seed=7 + rng)
            dets, _, _ = detect(img, INTR, POSE)
            if not dets:
                continue
            d = dets[0]
            if not d.type_trustworthy:
                uncertain += 1
            elif d.type == name:
                right += 1
            else:
                wrong += 1
                print("    WRONG: %s at %d in classified %s (r %.1f px, pred %.1f, sep %.1f)"
                      % (name, rng, d.type, d.radius_px, d.predicted_radius_px, d.separation_px))
    total = right + wrong
    print("  %d correct, %d wrong, %d flagged uncertain" % (right, wrong, uncertain))
    check("type classification at least 80%% correct where it claims confidence",
          total == 0 or right / float(total) >= 0.80,
          "%.0f%%" % (100 * right / float(total)) if total else "no confident calls")


def test_timing():
    print()
    print("=" * 78)
    print("6. Cost per frame (on this laptop -- the Control Hub is far slower)")
    print("=" * 78)
    import time
    img, _ = render_scene(INTR, POSE, [(30.0, 0.0, POLLEN_D), (60.0, 12.0, NECTAR_D)], seed=9)
    p = DetectParams()
    detect(img, INTR, POSE, p)                      # warm up
    t0 = time.perf_counter()
    n = 60
    for _ in range(n):
        detect(img, INTR, POSE, p)
    dt = (time.perf_counter() - t0) / n
    print("  %.2f ms per 640x400 frame here (%.0f fps equivalent)" % (dt * 1000, 1.0 / dt))
    print("  This is Python+OpenCV on an x86 laptop. The Java port on the Control Hub will differ;")
    print("  the useful signal is that the pipeline is a handful of O(pixels) passes, with no Hough")
    print("  transform and no per-pixel Python loops, so it should port to real-time on the Hub.")
    check("under 25 ms per frame on this machine", dt < 0.025, "%.1f ms" % (dt * 1000))


def main():
    global SAVE, OUTDIR
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", action="store_true", help="write annotated PNGs")
    ap.add_argument("--outdir", default="captures")
    args = ap.parse_args()
    SAVE, OUTDIR = args.save, args.outdir

    print()
    print("Mono ball detector, synthetic validation")
    print("Intrinsics PROVISIONAL (fx=%.0f at %dx%d, 70 deg HFOV assumed) -- camera not yet"
          % (INTR.fx, INTR.width, INTR.height))
    print("calibrated, so absolute inches will shift. Camera %.0f in up, pitched %.0f deg down."
          % (POSE.height_in, math.degrees(POSE.pitch_rad)))

    test_single_ball()
    test_many_balls()
    test_distractor_rejection()
    test_known_limitation()
    test_range_accuracy()
    test_type_classification()
    test_timing()

    print()
    print("=" * 78)
    print("%d passed, %d failed" % (PASS, FAIL))
    print("=" * 78)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
