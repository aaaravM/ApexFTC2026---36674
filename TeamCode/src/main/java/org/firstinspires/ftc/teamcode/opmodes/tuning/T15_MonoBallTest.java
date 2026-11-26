package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;
import org.firstinspires.ftc.teamcode.vision.MonoBallProcessor;
import org.firstinspires.ftc.teamcode.vision.MonoBallTracker;

import java.util.List;

/**
 * Bring-up and tuning for the monochrome ball detector. <b>Run this first.</b>
 *
 * <h2>It is built to tell you what is wrong, not just whether it works</h2>
 * Every tunable in the detector was set against SYNTHETIC images, so the first run on real hardware
 * will be wrong in some way. What matters is how fast you can tell WHICH way. Three things here are
 * for that:
 *
 * <ul>
 *   <li><b>The camera opens by fallback, not by assumption.</b> It lists every webcam in the robot
 *       configuration and tries five resolutions until one streams. A wrong camera name or an
 *       unsupported mode gives you a readable report instead of a crash or a silent no-op.</li>
 *   <li><b>Per-stage reject counts.</b> "It sees nothing" has half a dozen unrelated causes that
 *       look identical. The stage that ate everything names the cause -- see
 *       {@link MonoBallProcessor#STAGE_NAMES}.</li>
 *   <li><b>X dumps a real frame to the Hub.</b> Image, mask, and a sidecar recording every
 *       parameter and result. Pull them with
 *       {@code adb pull /sdcard/FIRST/ballcam C:\FTC-backup\ballcam} and the detector can then be
 *       tuned offline against actual images rather than by squinting at telemetry.</li>
 * </ul>
 *
 * <h2>Check these in order</h2>
 * <ol>
 *   <li><b>HORIZON.</b> The yellow line on the camera stream must sit on the real floor-to-wall
 *       junction. This is the cheapest and most important check here: it validates cy, the camera
 *       pitch and its SIGN, and the whole camera-to-world rotation at once, with no checkerboard.
 *       If the line is wrong, nothing downstream can be right. Calibration cannot do this -- its
 *       four numbers cannot be judged by eye.</li>
 *   <li><b>RESOLUTION.</b> Does "actual" match what was requested? A USB camera substitutes modes
 *       silently, and fx scales with width.</li>
 *   <li><b>CALIBRATION.</b> If it says UNCALIBRATED, ranges are coming from the lens datasheet
 *       rather than a measurement -- roughly right, not trustworthy. Run
 *       {@code tools/vision/calibrate.py}.</li>
 *   <li><b>STAGES.</b> With a ball in view, where do candidates die?</li>
 *   <li><b>RANGE.</b> Tape-measure a ball and compare. This is the measurement that matters.</li>
 * </ol>
 *
 * <h2>Measuring SHADOW_BIAS_FRAC, the one number that cannot be guessed</h2>
 * A ball on the tiles casts a shadow that the dark-polarity threshold claims and closing welds on,
 * so the blob reads bigger than the ball. Uncorrected this measured 45% large on synthetic frames,
 * which both misclassified every POLLEN and rejected real balls outright.
 *
 * <p>Put ONE ball at a tape-measured range and compare "r meas" against "r pred". Nudge with the
 * dpad until they agree, then repeat at two more distances -- if one value fits all three, it is
 * real; if each distance wants a different value, the focal length is wrong, not the bias. Write the
 * number into {@link MonoBallProcessor#SHADOW_BIAS_FRAC}; nothing here is saved.
 *
 * <h2>Controls</h2>
 * <pre>
 *   dpad up/down     SHADOW_BIAS_FRAC  +/- 0.01
 *   dpad right/left  RADIUS_TOL_FRAC   +/- 0.02   (the main accept/reject gate)
 *   bumpers R/L      ADAPTIVE_C        +/- 1      (raise to reject noise)
 *   X                dump this frame to the Hub
 *   Y                toggle exposure short/auto
 *   A                toggle streaming (gives the CPU back)
 *   B                reset every tunable to its committed default
 * </pre>
 */
@TeleOp(name = "T15 Mono Ball Test", group = "Tuning")
public class T15_MonoBallTest extends LinearOpMode {

    /**
     * Camera name as it appears in the robot configuration. If this is wrong the OpMode prints
     * every webcam it CAN see, so the right name is one glance away.
     */
    private static final String CAMERA_NAME = "Webcam 1";

    /** Lens height above the tiles, inches. MEASURE THIS -- range accuracy scales with it. */
    private static final double CAMERA_HEIGHT_IN = 16.0;

    /**
     * Camera tilt, <b>DOWN-positive</b> (opposite to {@code RobotConstants.Camera.PITCH_DEG}).
     *
     * <p>MEASURE THIS with a phone level app. It is the dominant error term in the ground-plane
     * range model, and its SIGN is the one mistake that cannot be corrected afterwards.
     *
     * <p>Tilting down is also worth doing for its own sake. Level, the floor only fills the bottom
     * half of the image and the nearest visible floor is about 21 in away at a 10.5 in lens. At
     * 18 degrees down the blind ring shrinks and most of the frame becomes usable floor.
     */
    private static final double CAMERA_PITCH_DOWN_DEG = -12.0;

    private static final double BIAS_STEP = 0.01;
    private static final double TOL_STEP = 0.02;

    @Override
    public void runOpMode() {
        final double defBias = MonoBallProcessor.SHADOW_BIAS_FRAC;
        final int defC = MonoBallProcessor.ADAPTIVE_C;
        final double defTol = MonoBallProcessor.RADIUS_TOL_FRAC;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);   // SDK default is 250 ms, too slow to tune on

        telemetry.addLine("Opening camera (trying 5 resolutions)...");
        telemetry.update();

        MonoBallTracker.OpenResult open = MonoBallTracker.openWithFallback(
                hardwareMap, CAMERA_NAME, MonoBallTracker.FALLBACK_MODES,
                CAMERA_HEIGHT_IN, CAMERA_PITCH_DOWN_DEG, 4000);

        if (!open.ok()) {
            telemetry.addLine("CAMERA DID NOT OPEN");
            telemetry.addLine();
            for (String line : open.log.split("\n")) telemetry.addLine(line);
            telemetry.addLine();
            telemetry.addLine("This OpMode asked for \"" + CAMERA_NAME + "\". If the list above");
            telemetry.addLine("shows a different name, change CAMERA_NAME in this file.");
            telemetry.update();
            waitForStart();
            return;
        }

        MonoBallTracker tracker = open.tracker;
        MonoBallProcessor proc = tracker.getProcessor();
        tracker.setManualExposure();

        for (String line : open.log.split("\n")) telemetry.addLine(line);
        telemetry.addLine();
        telemetry.addLine("Resolution: " + tracker.getActualResolution());
        telemetry.addLine(proc.getCalibrationSource());
        telemetry.addLine();
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();

        boolean streaming = true;
        boolean shortExposure = true;
        boolean upWas = false, downWas = false, leftWas = false, rightWas = false;
        boolean rbWas = false, lbWas = false, xWas = false, yWas = false;
        boolean aWas = false, bWas = false;

        while (opModeIsActive()) {
            // ---- tunables ----------------------------------------------------------------
            if (gamepad1.dpad_up && !upWas) MonoBallProcessor.SHADOW_BIAS_FRAC =
                    Math.min(0.60, MonoBallProcessor.SHADOW_BIAS_FRAC + BIAS_STEP);
            upWas = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !downWas) MonoBallProcessor.SHADOW_BIAS_FRAC =
                    Math.max(0.0, MonoBallProcessor.SHADOW_BIAS_FRAC - BIAS_STEP);
            downWas = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !rightWas) MonoBallProcessor.RADIUS_TOL_FRAC =
                    Math.min(1.0, MonoBallProcessor.RADIUS_TOL_FRAC + TOL_STEP);
            rightWas = gamepad1.dpad_right;
            if (gamepad1.dpad_left && !leftWas) MonoBallProcessor.RADIUS_TOL_FRAC =
                    Math.max(0.02, MonoBallProcessor.RADIUS_TOL_FRAC - TOL_STEP);
            leftWas = gamepad1.dpad_left;

            if (gamepad1.right_bumper && !rbWas) MonoBallProcessor.ADAPTIVE_C++;
            rbWas = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lbWas)
                MonoBallProcessor.ADAPTIVE_C = Math.max(1, MonoBallProcessor.ADAPTIVE_C - 1);
            lbWas = gamepad1.left_bumper;

            if (gamepad1.x && !xWas) proc.requestDump();
            xWas = gamepad1.x;

            if (gamepad1.y && !yWas) {
                shortExposure = !shortExposure;
                if (shortExposure) {
                    tracker.setManualExposure();
                } else {
                    ExposureControl exp = tracker.getVisionPortal()
                            .getCameraControl(ExposureControl.class);
                    if (exp != null) exp.setMode(ExposureControl.Mode.Auto);
                }
            }
            yWas = gamepad1.y;

            if (gamepad1.a && !aWas) {
                streaming = !streaming;
                if (streaming) tracker.resumeStreaming(); else tracker.stopStreaming();
            }
            aWas = gamepad1.a;

            if (gamepad1.b && !bWas) {
                MonoBallProcessor.SHADOW_BIAS_FRAC = defBias;
                MonoBallProcessor.ADAPTIVE_C = defC;
                MonoBallProcessor.RADIUS_TOL_FRAC = defTol;
            }
            bWas = gamepad1.b;

            // ---- telemetry ---------------------------------------------------------------
            BallGeometry.Intrinsics intr = proc.getIntrinsics();

            telemetry.addLine(tracker.getHealthLine());
            if (proc.isCalibrationFallback()) {
                telemetry.addLine("!! UNCALIBRATED: ranges come from the lens datasheet, not a");
                telemetry.addLine("!! measurement. Run tools/vision/calibrate.py.");
            }
            telemetry.addLine();

            telemetry.addData("Stages", proc.getStageSummary());
            if (proc.getRejectCount() > 0) {
                telemetry.addData("Last reject", proc.getLastRejectReason());
            }
            telemetry.addData("Horizon row", "%.0f of %d  (balls must be BELOW; check the line",
                    proc.getHorizonRow(), intr == null ? 0 : intr.height);
            telemetry.addLine("             sits on the real floor/wall junction)");
            telemetry.addLine();

            List<MonoBallProcessor.Sighting> seen = proc.getSightings();
            if (seen.isEmpty()) {
                telemetry.addLine("-- nothing accepted --");
                int[] c = proc.getStageCounts();
                if (c[0] == 0) {
                    telemetry.addLine("No contours at all: thresholds, lighting, or the lens cap.");
                } else {
                    telemetry.addLine("Contours exist but all were filtered. Read 'Stages' above:");
                    telemetry.addLine("  notRound       -> ball welded to its shadow, or block too small");
                    telemetry.addLine("  aboveHorizon   -> aimed too high, or the pitch sign is wrong");
                    telemetry.addLine("  radiusMismatch -> wrong fx or SHADOW_BIAS, not thresholds");
                }
            } else {
                telemetry.addLine("  type     range   bearing   r meas  r pred  consist");
                for (MonoBallProcessor.Sighting s : seen) {
                    telemetry.addLine(String.format(
                            "  %-8s %5.1f\"  %+6.1f\u00b0  %6.1f  %6.1f  %5.2f%s%s",
                            s.type == BallType.POLLEN ? "POLLEN" : "NECTAR",
                            s.rangeIn, Math.toDegrees(s.bearingFromAxisRad),
                            s.radiusPx, s.predictedRadiusPx, s.consistency,
                            s.typeTrustworthy ? "" : " ?",
                            s.airborne ? " AIR" : ""));
                }
                telemetry.addLine();
                MonoBallTracker.Ball closest = tracker.getClosest(0.0);
                if (closest != null) telemetry.addData("Closest, robot frame", closest.toString());
            }

            telemetry.addLine();
            telemetry.addData("Dumps (X)", "%d  %s", proc.getDumpCount(), proc.getLastDumpPath());
            telemetry.addData("SHADOW_BIAS (dpad u/d)", "%.2f%s",
                    MonoBallProcessor.SHADOW_BIAS_FRAC,
                    Math.abs(MonoBallProcessor.SHADOW_BIAS_FRAC - defBias) > 1e-9
                            ? "  *** CHANGED, NOT SAVED ***" : "");
            telemetry.addData("RADIUS_TOL (dpad l/r)", "%.1fpx + %.2f*pred",
                    MonoBallProcessor.RADIUS_TOL_PX, MonoBallProcessor.RADIUS_TOL_FRAC);
            telemetry.addData("ADAPTIVE_C (bumpers)", MonoBallProcessor.ADAPTIVE_C);
            telemetry.addData("ADAPTIVE_BLOCK", "%d (auto from width)", proc.getEffectiveBlock());
            telemetry.addData("Exposure (Y)", shortExposure ? "SHORT" : "AUTO");
            telemetry.addData("Streaming (A)", streaming ? "on" : "OFF");
            if (intr != null) {
                telemetry.addData("fx / fy", "%.1f / %.1f", intr.fx, intr.fy);
                telemetry.addData("cx / cy", "%.1f / %.1f", intr.cx, intr.cy);
                // Physical cross-check: the datasheet's 70 deg HFOV implies this fx. A calibration
                // can show a respectable reprojection error and still have fx badly wrong.
                double expect = MonoBallProcessor.expectedFxForWidth(intr.width);
                double offBy = 100.0 * (intr.fx - expect) / expect;
                telemetry.addData("fx vs lens spec", "%.0f expected, %+.0f%%%s", expect, offBy,
                        Math.abs(offBy) > 15 ? "  <-- SUSPECT" : "");
            }
            telemetry.addLine("B = reset tunables. Nothing here is saved to the code.");
            telemetry.update();
        }

        tracker.close();
    }
}
