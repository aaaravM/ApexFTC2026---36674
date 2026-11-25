#!/usr/bin/env python3
"""
Tune the ball detector offline against REAL frames pulled off the Control Hub.

================================================================================================
THE PROBLEM THIS SOLVES
================================================================================================
Every tunable in the detector was set against synthetic renders.  Lambertian shading is not real
shading: no scuffed tiles, no venue lighting, no other robot in frame, and no infrared -- the OV9281
has no IR filter and the datasheet calls it explicitly IR-sensitive, so gym lights and daylight do
not look the way they would on a colour camera.  SHADOW_BIAS_FRAC in particular cannot be derived at
all; it has to be measured.

The alternative is standing at the robot nudging numbers on a gamepad and squinting at telemetry.
That is slow, it produces nothing anyone else can check, and it cannot be redone when the lighting
changes.

So: T15 dumps frames (press X), you pull them, and this fits the constants properly.

    adb pull /sdcard/FIRST/ballcam C:\\FTC-backup\\ballcam
    python tools/vision/tune_from_dump.py --dir C:\\FTC-backup\\ballcam

================================================================================================
MEASURE THE TRUTH, OR THIS CANNOT HELP YOU
================================================================================================
A frame on its own tells you what the detector DID, not whether it was right.  To fit anything you
need ground truth, and the only source is a tape measure.

Open each frame's `_info.txt` and add one line per ball actually in shot:

    truth 36 POLLEN
    truth 52.5 NECTAR

Range in inches from the LENS to the ball centre, horizontally.  Then --sweep can solve for the
constants that make the detector agree with the tape.  Without truth lines this still re-runs the
detector and draws overlays, which is useful for seeing WHY something was rejected, but it cannot
fit anything.

================================================================================================
WHAT --sweep ACTUALLY FITS, AND WHAT IT CANNOT
================================================================================================
It grids over SHADOW_BIAS_FRAC and reports the value minimising range error against truth.

The important part is the DIAGNOSTIC, not the number.  If one bias value fits every distance, the
bias is real and you are done.  If each distance wants a different bias, then the error is not a
constant fraction of the radius -- which means the focal length is wrong, and no amount of bias
tuning will fix it.  The script says which case you are in, because fixing the wrong one is the
expensive mistake here.

Note also that range from the ground plane does NOT depend on the radius at all, so a bias error
does not move the reported range -- it moves the TYPE decision and the accept/reject gate.  Range
error against truth is therefore evidence about fx, pitch and camera height; radius error is
evidence about the bias.  The sweep reports both separately for exactly that reason.
"""

import argparse
import glob
import math
import os
import re
import sys

import cv2
import numpy as np

from ball_geometry import CameraPose, Intrinsics, predicted_radius_px
from ball_detect import DetectParams, detect, draw_overlay, render_scene, POLLEN_D, NECTAR_D

# Truth-line type name -> diameter. Used to compute the radius a ball of the TAPE-MEASURED type
# would subtend, which is the only honest target for a radius fit -- see radius_error_vs_truth.
TRUTH_DIAMETER = {"POLLEN": POLLEN_D, "NECTAR": NECTAR_D,
                  "NECTAR_RED": NECTAR_D, "NECTAR_BLUE": NECTAR_D}


def radius_error_vs_truth(det, truth_rng, truth_name, intr, pose):
    """|measured radius - radius a TRUTH-type ball would subtend at the TRUTH range|, in pixels.

    Why not just use det.predicted_radius_px: that is the prediction for whichever type the detector
    CHOSE. If the bias is wrong enough to flip the type, the detector then compares against the
    other ball's radius and the error collapses to near zero -- so a sweep minimising it happily
    converges on a bias that gets the type WRONG. The first run of this script did exactly that: it
    reported a 'best' bias of 0.30 whose per-distance fit at 72 in was 0.09, purely because POLLEN
    had been misread as NECTAR and was being scored against the wrong target.

    Anchoring on the tape-measured type and range removes both degrees of freedom. Returns None when
    the truth line has no usable type.
    """
    D = TRUTH_DIAMETER.get((truth_name or "").upper())
    if D is None:
        return None
    pred = predicted_radius_px(truth_rng, intr, pose, D)
    if pred is None or pred <= 0:
        return None
    return abs(det.radius_px - pred)

NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


class Dump:
    """One dumped frame plus the parameters and truth that go with it."""

    def __init__(self, gray, info_path, meta, truth):
        self.gray = gray
        self.info_path = info_path
        self.meta = meta
        self.truth = truth          # [(range_in, type_name)]
        self.name = os.path.basename(info_path).replace("_info.txt", "")

    def intrinsics(self):
        w, h = self.meta["resolution"]
        return Intrinsics(self.meta["fx"], self.meta["fy"],
                          self.meta["cx"], self.meta["cy"], w, h)

    def pose(self):
        return CameraPose(self.meta["cameraHeightIn"], self.meta["pitchDownDeg"])

    def params(self, **override):
        p = DetectParams()
        for k_sidecar, k_param in [
            ("BLUR_KSIZE", "blur_ksize"), ("ADAPTIVE_BLOCK", "adaptive_block"),
            ("ADAPTIVE_C", "adaptive_c"), ("MORPH_KSIZE", "morph_ksize"),
            ("MIN_RADIUS_PX", "min_radius_px"), ("MAX_RADIUS_PX", "max_radius_px"),
            ("MIN_CIRCULARITY", "min_circularity"), ("MIN_FILL", "min_fill"),
            ("SHADOW_BIAS_FRAC", "shadow_bias_frac"), ("RADIUS_TOL_FRAC", "radius_tol_frac"),
            ("MAX_RANGE_IN", "max_range_in"), ("CONSISTENCY_MAX", "consistency_max"),
        ]:
            if k_sidecar in self.meta:
                v = self.meta[k_sidecar]
                setattr(p, k_param, int(v) if isinstance(getattr(p, k_param), int) else float(v))
        for k, v in override.items():
            setattr(p, k, v)
        return p


def parse_info(path):
    """Read a sidecar. Returns (meta dict, truth list)."""
    meta, truth = {}, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("truth"):
                m = re.match(r"truth\s+(%s)\s*(\w+)?" % NUM, line)
                if m:
                    truth.append((float(m.group(1)), (m.group(2) or "").upper()))
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            key = parts[0]
            if key == "resolution":
                m = re.match(r"(\d+)x(\d+)", parts[1])
                if m:
                    meta["resolution"] = (int(m.group(1)), int(m.group(2)))
            elif key == "fx" and parts[0] == "fx":
                # "fx fy  <a> <b>"
                nums = re.findall(NUM, line)
                if len(nums) >= 2:
                    meta["fx"], meta["fy"] = float(nums[0]), float(nums[1])
            elif key == "cx":
                nums = re.findall(NUM, line)
                if len(nums) >= 2:
                    meta["cx"], meta["cy"] = float(nums[0]), float(nums[1])
            else:
                nums = re.findall(NUM, parts[1])
                if nums:
                    try:
                        meta[key] = float(nums[0])
                    except ValueError:
                        pass
    return meta, truth


def load_dumps(d):
    out = []
    for info in sorted(glob.glob(os.path.join(d, "*_info.txt"))):
        stem = info.replace("_info.txt", "")
        gray_path = stem + "_gray.png"
        if not os.path.exists(gray_path):
            print("  skip (no _gray.png) %s" % os.path.basename(info))
            continue
        gray = cv2.imread(gray_path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print("  skip (unreadable image) %s" % gray_path)
            continue
        meta, truth = parse_info(info)
        need = ["resolution", "fx", "fy", "cx", "cy", "cameraHeightIn", "pitchDownDeg"]
        missing = [k for k in need if k not in meta]
        if missing:
            print("  skip (sidecar missing %s) %s" % (missing, os.path.basename(info)))
            continue
        if (gray.shape[1], gray.shape[0]) != meta["resolution"]:
            print("  WARNING %s: image is %dx%d but sidecar says %dx%d -- using the image size"
                  % (os.path.basename(gray_path), gray.shape[1], gray.shape[0],
                     meta["resolution"][0], meta["resolution"][1]))
            meta["resolution"] = (gray.shape[1], gray.shape[0])
        out.append(Dump(gray, info, meta, truth))
    return out


def match_truth(dets, truth):
    """Pair each truth entry with the nearest detection by range. Returns [(t, det or None)]."""
    unused = list(dets)
    pairs = []
    for rng, name in sorted(truth):
        best, bestd = None, 1e18
        for d in unused:
            e = abs(d.range_in - rng)
            if e < bestd:
                best, bestd = d, e
        # Generous gate: we are diagnosing, not scoring. 40% or 12 in, whichever is larger.
        if best is not None and bestd <= max(12.0, 0.4 * rng):
            pairs.append(((rng, name), best))
            unused.remove(best)
        else:
            pairs.append(((rng, name), None))
    return pairs, unused


def report(dumps, outdir, save):
    print()
    print("=" * 78)
    print("RE-RUN WITH EACH FRAME'S OWN RECORDED PARAMETERS")
    print("=" * 78)
    any_truth = False
    for d in dumps:
        intr, pose, p = d.intrinsics(), d.pose(), d.params()
        dets, mask, rejects = detect(d.gray, intr, pose, p, want_rejects=True)
        print()
        print("%s   %dx%d  fx=%.1f  h=%.1f\"  pitch=%.1f deg down  %s"
              % (d.name, intr.width, intr.height, intr.fx, pose.height_in,
                 math.degrees(pose.pitch_rad),
                 "" if "calibration" not in d.meta else ""))
        print("  accepted %d, rejected %d" % (len(dets), len(rejects)))
        for s in dets:
            print("    %-7s range %6.1f\"  bearing %+6.1f deg  r %5.1f (pred %5.1f)  consist %.2f%s"
                  % (s.type, s.range_in, math.degrees(s.bearing_rad), s.radius_px,
                     s.predicted_radius_px, s.consistency,
                     "" if s.type_trustworthy else "  type uncertain"))
        for (u, v, r, why) in rejects[:6]:
            print("    rejected r=%5.1f at (%4.0f,%4.0f): %s" % (r, u, v, why))
        if len(rejects) > 6:
            print("    ... and %d more rejects" % (len(rejects) - 6))

        if d.truth:
            any_truth = True
            pairs, extra = match_truth(dets, d.truth)
            print("  vs TAPE MEASURE:")
            for (rng, name), det in pairs:
                if det is None:
                    print("    truth %6.1f\" %-7s -> NOT DETECTED" % (rng, name))
                else:
                    print("    truth %6.1f\" %-7s -> got %6.1f\" (%+.1f\", %+.1f%%) type %s%s"
                          % (rng, name, det.range_in, det.range_in - rng,
                             100 * (det.range_in - rng) / rng, det.type,
                             "" if (not name or det.type.startswith(name[:6]))
                             else "  <-- TYPE WRONG"))
            if extra:
                print("    %d detection(s) with no truth entry (false positives?)" % len(extra))
        else:
            print("  no truth lines in the sidecar -- add 'truth <inches> <TYPE>' to fit anything")

        if save:
            os.makedirs(outdir, exist_ok=True)
            cv2.imwrite(os.path.join(outdir, d.name + "_overlay.png"),
                        draw_overlay(d.gray, dets, rejects))
            cv2.imwrite(os.path.join(outdir, d.name + "_mask.png"), mask)
    if save:
        print()
        print("overlays written to %s" % outdir)
    return any_truth


def sweep(dumps):
    """Grid over SHADOW_BIAS_FRAC against tape-measured truth."""
    withtruth = [d for d in dumps if d.truth]
    if not withtruth:
        print()
        print("Cannot sweep: no frame has 'truth' lines. Measure the balls with a tape and add")
        print("    truth <inches> <POLLEN|NECTAR>")
        print("to the _info.txt files, one line per ball in shot.")
        return

    print()
    print("=" * 78)
    print("SWEEP: SHADOW_BIAS_FRAC against %d frame(s) with truth" % len(withtruth))
    print("=" * 78)
    print("  %-7s %-8s %-10s %-11s %s"
          % ("bias", "found", "med|dR| in", "med|dr| px", "types right"))

    best = None
    rows = []
    for bias in [i / 100.0 for i in range(0, 41, 2)]:
        rng_errs, rad_errs, found, total, types_ok, types_called = [], [], 0, 0, 0, 0
        for d in withtruth:
            intr, pose = d.intrinsics(), d.pose()
            p = d.params(shadow_bias_frac=bias)
            dets, _, _ = detect(d.gray, intr, pose, p)
            pairs, _ = match_truth(dets, d.truth)
            for (truth_rng, truth_name), det in pairs:
                total += 1
                if det is None:
                    continue
                found += 1
                rng_errs.append(abs(det.range_in - truth_rng))
                re_ = radius_error_vs_truth(det, truth_rng, truth_name, intr, pose)
                if re_ is not None:
                    rad_errs.append(re_)
                if truth_name:
                    types_called += 1
                    if det.type.startswith(truth_name[:6]):
                        types_ok += 1
        if not rng_errs:
            rows.append((bias, 0, float("nan"), float("nan"), 0, 0))
            continue
        mr = float(np.median(rng_errs))
        mrad = float(np.median(rad_errs)) if rad_errs else float("nan")
        rows.append((bias, found, mr, mrad, types_ok, types_called))
        print("  %-7.2f %d/%-6d %-10.2f %-11.2f %d/%d"
              % (bias, found, total, mr, mrad, types_ok, types_called))
        # Rank on radius agreement: that is what the bias actually controls.
        score = (-found, mrad if not math.isnan(mrad) else 1e9)
        if best is None or score < best[0]:
            best = (score, bias, mr, mrad, found, total, types_ok, types_called)

    if best is None:
        print("  nothing detected at any bias -- the problem is upstream of the bias")
        return

    _, bias, mr, mrad, found, total, t_ok, t_called = best
    print()
    print("BEST: SHADOW_BIAS_FRAC = %.2f" % bias)
    print("  found %d/%d truth balls, median radius error %.2f px, median range error %.2f in"
          % (found, total, mrad, mr))
    if t_called:
        print("  type correct %d/%d" % (t_ok, t_called))
    print()

    # The diagnostic that matters more than the number.
    print("IS THE BIAS REAL, OR IS fx WRONG?")
    per_distance = []
    for d in withtruth:
        intr, pose = d.intrinsics(), d.pose()
        for truth_rng, _name in d.truth:
            local_best, local_err = None, 1e18
            for b in [i / 100.0 for i in range(0, 41, 1)]:
                dets, _, _ = detect(d.gray, intr, pose, d.params(shadow_bias_frac=b))
                pairs, _ = match_truth(dets, [(truth_rng, _name)])
                (_, det) = pairs[0]
                if det is None:
                    continue
                e = radius_error_vs_truth(det, truth_rng, _name, intr, pose)
                if e is not None and e < local_err:
                    local_best, local_err = b, e
            if local_best is not None:
                per_distance.append((truth_rng, local_best))

    if len(per_distance) < 2:
        print("  need at least two different measured distances to tell these apart.")
        return

    per_distance.sort()
    biases = [b for _, b in per_distance]
    spread = max(biases) - min(biases)
    for rng, b in per_distance:
        print("    at %6.1f\"  best bias %.2f" % (rng, b))
    print()
    if spread <= 0.06:
        print("  Spread is %.2f -- CONSISTENT. The bias is a real, range-independent effect." % spread)
        print("  Put %.2f into MonoBallProcessor.SHADOW_BIAS_FRAC and ball_detect's default." % bias)
    else:
        print("  Spread is %.2f -- NOT consistent across distance." % spread)
        print("  A contact shadow is a roughly FIXED FRACTION of the radius, so it cannot produce a")
        print("  range-dependent bias. This points at the FOCAL LENGTH instead: fx is a multiplier")
        print("  on predicted radius at every range, and getting it wrong looks exactly like this.")
        print("  Calibrate (tools/vision/calibrate.py) before tuning the bias any further.")


def make_test_dump(outdir):
    """Write a synthetic dump in the real sidecar format, to test this script end to end.

    Also serves as living documentation of the format T15 writes.
    """
    os.makedirs(outdir, exist_ok=True)
    intr = Intrinsics(914.0, 914.0, 640.0, 400.0, 1280, 800)
    pose = CameraPose(10.5, 18.0)
    cases = [(30.0, POLLEN_D, "POLLEN"), (48.0, NECTAR_D, "NECTAR"), (72.0, POLLEN_D, "POLLEN")]

    for i, (rng, D, name) in enumerate(cases):
        img, truth = render_scene(intr, pose, [(rng, 0.0, D)], seed=100 + i)
        stem = os.path.join(outdir, "frame_%03d" % i)
        cv2.imwrite(stem + "_gray.png", img)
        p = DetectParams()
        with open(stem + "_info.txt", "w") as f:
            f.write("# SYNTHETIC test dump, written by tune_from_dump.py --make-test-dump\n")
            f.write("# Same format T15 writes on the Hub.\n\n")
            f.write("resolution     %dx%d\n" % (intr.width, intr.height))
            f.write("fx fy          %.4f %.4f\n" % (intr.fx, intr.fy))
            f.write("cx cy          %.4f %.4f\n" % (intr.cx, intr.cy))
            f.write("cameraHeightIn %.4f\n" % pose.height_in)
            f.write("pitchDownDeg   %.4f\n" % math.degrees(pose.pitch_rad))
            f.write("\n")
            f.write("BLUR_KSIZE        %d\n" % p.blur_ksize)
            f.write("ADAPTIVE_BLOCK    %d\n" % ((intr.width // 10) | 1))
            f.write("ADAPTIVE_C        %d\n" % p.adaptive_c)
            f.write("MORPH_KSIZE       %d\n" % p.morph_ksize)
            f.write("MIN_RADIUS_PX     %.4f\n" % p.min_radius_px)
            f.write("MAX_RADIUS_PX     %.4f\n" % p.max_radius_px)
            f.write("MIN_CIRCULARITY   %.4f\n" % p.min_circularity)
            f.write("MIN_FILL          %.4f\n" % p.min_fill)
            f.write("SHADOW_BIAS_FRAC  %.4f\n" % p.shadow_bias_frac)
            f.write("RADIUS_TOL_FRAC   %.4f\n" % p.radius_tol_frac)
            f.write("MAX_RANGE_IN      %.4f\n" % p.max_range_in)
            f.write("CONSISTENCY_MAX   %.4f\n" % p.consistency_max)
            f.write("\n")
            f.write("truth %.1f %s\n" % (rng, name))
    print("wrote %d synthetic frames with truth lines to %s" % (len(cases), outdir))
    print("now: python tools/vision/tune_from_dump.py --dir %s --sweep" % outdir)


def main():
    ap = argparse.ArgumentParser(description="Tune the ball detector from dumped Hub frames.")
    ap.add_argument("--dir", help="directory of frame_*_gray.png + frame_*_info.txt")
    ap.add_argument("--sweep", action="store_true", help="fit SHADOW_BIAS_FRAC against truth")
    ap.add_argument("--save-overlays", action="store_true")
    ap.add_argument("--outdir", default="dump_overlays")
    ap.add_argument("--make-test-dump", metavar="DIR",
                    help="write synthetic frames in the sidecar format, to test this script")
    a = ap.parse_args()

    if a.make_test_dump:
        make_test_dump(a.make_test_dump)
        return 0
    if not a.dir:
        ap.print_help()
        print()
        print("Typical use:")
        print("  adb pull /sdcard/FIRST/ballcam C:\\FTC-backup\\ballcam")
        print("  # add 'truth <inches> <TYPE>' lines to each _info.txt")
        print("  python tools/vision/tune_from_dump.py --dir C:\\FTC-backup\\ballcam --sweep")
        return 1

    print("loading dumps from %s" % a.dir)
    dumps = load_dumps(a.dir)
    if not dumps:
        print("no usable dumps found.")
        return 1
    print("loaded %d frame(s)" % len(dumps))

    report(dumps, a.outdir, a.save_overlays)
    if a.sweep:
        sweep(dumps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
