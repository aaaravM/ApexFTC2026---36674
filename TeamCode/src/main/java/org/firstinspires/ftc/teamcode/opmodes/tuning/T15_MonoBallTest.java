package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;
import org.firstinspires.ftc.teamcode.vision.MonoBallProcessor;
import org.firstinspires.ftc.teamcode.vision.MonoBallTracker;

import java.util.List;

/**
 * Bring-up and tuning for the monochrome ball detector. Run this FIRST, before any auto-collect.
 *
 * <h2>Numbered T15 deliberately</h2>
 * T8 through T14 are taken on the turret and kicker branches. The old colour {@code
 * T8_BallCameraTest} is gone along with the colour {@code BallTracker} it drove, which could not
 * work on a mono sensor.
 *
 * <h2>What to check, in this order</h2>
 * <ol>
 *   <li><b>RESOLUTION.</b> Does "actual" match what was requested? A USB camera silently
 *       substitutes the nearest mode it supports. Probing this camera on a laptop found 640x400
 *       coming back as 640x480. If they differ, fix the request before trusting a single distance.</li>
 *   <li><b>CALIBRATION.</b> If it says UNCALIBRATED, the SDK had no entry matching this camera's
 *       USB vid/pid and hard-coded numbers are being scaled instead. Every range is then only as
 *       good as a guess. Run {@code tools/vision/calibrate.py} and install the generated
 *       {@code teamwebcamcalibrations.xml}.</li>
 *   <li><b>HORIZON.</b> The yellow line in the camera stream. Balls must be below it. If the floor
 *       barely appears below the line, tilt the camera further DOWN -- and note that mounting the
 *       camera HIGHER improves range accuracy proportionally.</li>
 *   <li><b>BLOBS vs ACCEPTED.</b> If blobs are found but nothing is accepted, read the reject
 *       reason: it names the filter that ate them. "matches no ball at Nin" almost always means
 *       SHADOW_BIAS or the calibration is wrong, not that the thresholds are wrong.</li>
 *   <li><b>RANGE.</b> Put a ball at a tape-measured distance and compare. This is the measurement
 *       that matters; everything else is in service of it.</li>
 * </ol>
 *
 * <h2>Measuring SHADOW_BIAS_FRAC, which is currently a placeholder</h2>
 * A ball on the tiles casts a shadow that the dark-polarity threshold claims and closing welds on,
 * so the blob reads bigger than the ball. The committed value came from synthetic renders and is a
 * placeholder. To measure it for real: put a single ball at a tape-measured range, read "r meas" and
 * "r pred" below, and adjust with the bumpers until they agree at two or three different distances.
 * Then put that number in {@link MonoBallProcessor#SHADOW_BIAS_FRAC}. Nothing here is saved.
 *
 * <h2>Controls</h2>
 * <pre>
 *   dpad up/down     SHADOW_BIAS_FRAC  +/- 0.01
 *   bumpers R/L      ADAPTIVE_C        +/- 1      (raise to reject noise)
 *   dpad right/left  ADAPTIVE_BLOCK    +/- 4      (must exceed a far ball's diameter in px)
 *   X / Y            RADIUS_TOL_FRAC   +/- 0.02   (the main accept/reject gate)
 *   A                toggle streaming (gives the CPU back)
 *   B                reset every tunable to its committed default
 * </pre>
 */
@TeleOp(name = "T15 Mono Ball Test", group = "Tuning")
public class T15_MonoBallTest extends LinearOpMode {

    /** Camera name. Change if the ball camera is configured under a different name. */
    private static final String CAMERA_NAME = "Webcam 1";

    /**
     * Request the sensor's NATIVE resolution.
     *
     * <p>1280x800 in MJPEG runs at 100-120 fps on this camera, so it is both the highest resolution
     * AND the highest framerate available -- there is no trade to make. (In YUY2, which is
     * VisionPortal's default, the same resolution manages only 10 fps; MonoBallTracker forces
     * MJPEG for exactly that reason.)
     *
     * <p>It is also the only mode whose field of view is documented: Arducam specifies 70 degrees
     * horizontal on the native 16:10 sensor. A 4:3 mode such as 640x480 may crop rather than scale,
     * which changes the FOV and moves the principal point, so calibration does not carry across.
     */
    private static final int REQ_WIDTH  = 1280;
    private static final int REQ_HEIGHT = 800;

    /** Lens height above the tiles, inches. MEASURE THIS -- range accuracy scales with it. */
    private static final double CAMERA_HEIGHT_IN = 10.5;

    /** Positive = tilted DOWN toward the floor. MEASURE THIS with a phone level app. */
    private static final double CAMERA_PITCH_DEG = 18.0;

    private static final double BIAS_STEP = 0.01;
    private static final double TOL_STEP  = 0.02;

    @Override
    public void runOpMode() {
        final double defBias = MonoBallProcessor.SHADOW_BIAS_FRAC;
        final int defC = MonoBallProcessor.ADAPTIVE_C;
        final int defBlock = MonoBallProcessor.ADAPTIVE_BLOCK;
        final double defTol = MonoBallProcessor.RADIUS_TOL_FRAC;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);   // SDK default is 250 ms, far too slow to tune on

        MonoBallTracker tracker;
        try {
            tracker = new MonoBallTracker(hardwareMap, CAMERA_NAME, REQ_WIDTH, REQ_HEIGHT,
                    CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG);
        } catch (Exception e) {
            // A missing or misnamed camera throws at init, which on the Driver Station reads like a
            // code problem rather than a configuration one. Say which it is.
            telemetry.addLine("FAILED to open camera \"" + CAMERA_NAME + "\"");
            telemetry.addLine(e.getClass().getSimpleName() + ": " + e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check the name in the Driver Station's robot configuration, under");
            telemetry.addLine("USB Devices. It must match CAMERA_NAME in this file exactly.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Opening camera...");
        telemetry.update();
        tracker.setManualExposure();

        MonoBallProcessor proc = tracker.getProcessor();

        telemetry.addLine("Requested " + REQ_WIDTH + "x" + REQ_HEIGHT
                + ", got " + tracker.getActualResolution());
        telemetry.addLine(proc.getCalibrationSource());
        telemetry.addLine();
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();

        boolean streaming = true;
        boolean upWas = false, downWas = false, rbWas = false, lbWas = false;
        boolean rightWas = false, leftWas = false, xWas = false, yWas = false;
        boolean aWas = false, bWas = false;

        while (opModeIsActive()) {
            // ---- tunables -------------------------------------------------------------------
            if (gamepad1.dpad_up && !upWas) MonoBallProcessor.SHADOW_BIAS_FRAC =
                    Math.min(0.60, MonoBallProcessor.SHADOW_BIAS_FRAC + BIAS_STEP);
            upWas = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !downWas) MonoBallProcessor.SHADOW_BIAS_FRAC =
                    Math.max(0.0, MonoBallProcessor.SHADOW_BIAS_FRAC - BIAS_STEP);
            downWas = gamepad1.dpad_down;

            if (gamepad1.right_bumper && !rbWas) MonoBallProcessor.ADAPTIVE_C++;
            rbWas = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lbWas)
                MonoBallProcessor.ADAPTIVE_C = Math.max(1, MonoBallProcessor.ADAPTIVE_C - 1);
            lbWas = gamepad1.left_bumper;

            if (gamepad1.dpad_right && !rightWas) MonoBallProcessor.ADAPTIVE_BLOCK += 4;
            rightWas = gamepad1.dpad_right;
            if (gamepad1.dpad_left && !leftWas)
                MonoBallProcessor.ADAPTIVE_BLOCK = Math.max(5, MonoBallProcessor.ADAPTIVE_BLOCK - 4);
            leftWas = gamepad1.dpad_left;

            if (gamepad1.x && !xWas) MonoBallProcessor.RADIUS_TOL_FRAC =
                    Math.min(1.0, MonoBallProcessor.RADIUS_TOL_FRAC + TOL_STEP);
            xWas = gamepad1.x;
            if (gamepad1.y && !yWas) MonoBallProcessor.RADIUS_TOL_FRAC =
                    Math.max(0.02, MonoBallProcessor.RADIUS_TOL_FRAC - TOL_STEP);
            yWas = gamepad1.y;

            if (gamepad1.a && !aWas) {
                streaming = !streaming;
                if (streaming) tracker.resumeStreaming(); else tracker.stopStreaming();
            }
            aWas = gamepad1.a;

            if (gamepad1.b && !bWas) {
                MonoBallProcessor.SHADOW_BIAS_FRAC = defBias;
                MonoBallProcessor.ADAPTIVE_C = defC;
                MonoBallProcessor.ADAPTIVE_BLOCK = defBlock;
                MonoBallProcessor.RADIUS_TOL_FRAC = defTol;
            }
            bWas = gamepad1.b;

            // ---- telemetry ------------------------------------------------------------------
            BallGeometry.Intrinsics intr = proc.getIntrinsics();

            telemetry.addLine(tracker.getHealthLine());
            if (proc.isCalibrationFallback()) {
                telemetry.addLine("!! UNCALIBRATED -- every distance below is only as good as a");
                telemetry.addLine("!! guessed focal length. Run tools/vision/calibrate.py.");
            }
            telemetry.addLine();

            telemetry.addData("Blobs / accepted / rejected", "%d / %d / %d",
                    proc.getBlobCount(), proc.getSightings().size(), proc.getRejectCount());
            if (proc.getRejectCount() > 0) {
                telemetry.addData("Last reject", proc.getLastRejectReason());
            }
            telemetry.addData("Horizon row", "%.0f  (balls must be BELOW this)",
                    proc.getHorizonRow());
            telemetry.addLine();

            List<MonoBallProcessor.Sighting> seen = proc.getSightings();
            if (seen.isEmpty()) {
                telemetry.addLine("-- nothing accepted --");
                if (proc.getBlobCount() == 0) {
                    telemetry.addLine("No round blobs at all: thresholds, lighting, or the lens cap.");
                } else {
                    telemetry.addLine("Blobs found but all filtered. Read 'Last reject' above.");
                }
            } else {
                telemetry.addLine("  type     range   bearing   r meas  r pred  consist");
                for (MonoBallProcessor.Sighting s : seen) {
                    telemetry.addLine(String.format("  %-8s %5.1f\"  %+6.1f°  %6.1f  %6.1f  %5.2f%s%s",
                            s.type == org.firstinspires.ftc.teamcode.util.BallType.POLLEN
                                    ? "POLLEN" : "NECTAR",
                            s.rangeIn, Math.toDegrees(s.bearingFromAxisRad),
                            s.radiusPx, s.predictedRadiusPx, s.consistency,
                            s.typeTrustworthy ? "" : " ?",
                            s.airborne ? " AIR" : ""));
                }
                telemetry.addLine();
                // Robot frame, with the turret assumed at zero: this OpMode has no turret.
                MonoBallTracker.Ball closest = tracker.getClosest(0.0);
                if (closest != null) telemetry.addData("Closest, robot frame", closest.toString());
            }

            telemetry.addLine();
            telemetry.addData("SHADOW_BIAS (dpad u/d)", "%.2f%s",
                    MonoBallProcessor.SHADOW_BIAS_FRAC,
                    Math.abs(MonoBallProcessor.SHADOW_BIAS_FRAC - defBias) > 1e-9
                            ? "  *** CHANGED, NOT SAVED ***" : "");
            telemetry.addData("ADAPTIVE_C (bumpers)", MonoBallProcessor.ADAPTIVE_C);
            telemetry.addData("ADAPTIVE_BLOCK (dpad l/r)", "%d%s",
                    proc.getEffectiveBlock(),
                    MonoBallProcessor.ADAPTIVE_BLOCK == 0 ? " (auto from width)" : " (override)");
            telemetry.addData("RADIUS_TOL (X / Y)", "%.2f", MonoBallProcessor.RADIUS_TOL_FRAC);
            telemetry.addData("Streaming (A)", streaming ? "on" : "OFF");
            if (intr != null) {
                telemetry.addData("fx / fy", "%.1f / %.1f", intr.fx, intr.fy);
                telemetry.addData("cx / cy", "%.1f / %.1f", intr.cx, intr.cy);
                // Physical cross-check: the datasheet's 70 deg HFOV implies this fx. A calibration
                // can show a respectable reprojection error and still have fx badly wrong, so
                // compare against the lens spec too.
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
