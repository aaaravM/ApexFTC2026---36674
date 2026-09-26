package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * STEP 2. Refines the two pod offsets. Run only after {@link T1_OdometryTest} passes everything.
 *
 * <h2>What the offsets do</h2>
 * Each pod sits somewhere other than the point whose position you want to report. When the robot
 * rotates, a pod that is off to one side is dragged sideways through an arc and reads a displacement
 * the robot's centre never made. The Pinpoint subtracts that out -- but only if you told it where
 * the pods are. Concretely, it computes:
 * <pre>
 *   forward velocity = (X pod reading) + omega * xOffset
 *   left velocity    = (Y pod reading) - omega * yOffset
 * </pre>
 * Offsets therefore affect <b>rotation only</b>. If driving in a straight line is inaccurate, the
 * offsets are not your problem -- go back to T1.
 *
 * <h2>The measurement, and why it is a HALF turn</h2>
 * Work out the position error that an offset mistake produces over a rotation of angle {@code T},
 * and you get:
 * <pre>
 *   field error = ( -dy*(1 - cos T) - dx*sin T ,  -dx*(1 - cos T) + dy*sin T )
 * </pre>
 * where {@code dx}, {@code dy} are how wrong each offset is. At {@code T = 360} degrees both terms
 * vanish: <b>a full turn hides the error completely</b>, which is why "spin it ten times and see if
 * it comes back to zero" does not work here. At {@code T = 180} degrees it reduces to a clean,
 * decoupled:
 * <pre>
 *   reported X = -2 * dy        reported Y = -2 * dx
 * </pre>
 * So one accurate half turn gives you both corrections directly:
 * <pre>
 *   new X offset = old X offset - (reported Y) / 2
 *   new Y offset = old Y offset - (reported X) / 2
 * </pre>
 * (Everything in millimetres, matching {@code RobotConstants.Odo}.)
 *
 * <p>The residual is periodic in the rotation angle, so 540 or 900 degrees gives the same reading
 * as 180 -- there is no accuracy to be gained by spinning more. What matters is that the half turn
 * is <b>exactly</b> 180 degrees and that the tracking point stays over one spot on the floor.
 *
 * <h2>Procedure</h2>
 * <ol>
 *   <li>Measure both offsets with a ruler first and put them in {@code RobotConstants.Odo}. This
 *       tuner corrects a small error; it will not rescue a wild guess.</li>
 *   <li>Mark the floor under your tracking point (drop a plumb line, or push a pin through tape).</li>
 *   <li>Run this OpMode. Press <b>A</b> to zero.</li>
 *   <li>Rotate the robot by hand to <b>exactly +180 degrees</b>, keeping the tracking point over
 *       the mark. Telemetry shows the live heading so you can land it precisely. Slowly.</li>
 *   <li>Press <b>B</b>. Read off the two suggested offsets, put them in the constants file,
 *       redeploy, and repeat once. The second pass should suggest almost no change.</li>
 * </ol>
 */
@TeleOp(name = "T2 Pinpoint Offset Tuner", group = "Tuning")
public class T2_PinpointOffsetTuner extends LinearOpMode {

    private static final double MM_PER_INCH = 25.4;

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Put the tracking point over a floor mark.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        double suggestedX = RobotConstants.Odo.X_POD_OFFSET_MM;
        double suggestedY = RobotConstants.Odo.Y_POD_OFFSET_MM;
        boolean captured = false;
        double capturedHeadingDeg = 0, capturedXmm = 0, capturedYmm = 0;

        // Track how far the reported position wanders during the turn. With correct offsets this
        // stays small at every heading, not just at the end.
        double maxDriftMm = 0;
        boolean aWasPressed = false, bWasPressed = false;

        while (opModeIsActive()) {
            odometry.update();

            double xMm = odometry.getX() * MM_PER_INCH;
            double yMm = odometry.getY() * MM_PER_INCH;
            double headingDeg = odometry.getHeadingDeg();
            maxDriftMm = Math.max(maxDriftMm, Math.hypot(xMm, yMm));

            if (gamepad1.a && !aWasPressed) {
                odometry.resetPoseAndCalibrateImu();
                maxDriftMm = 0;
                captured = false;
            }
            aWasPressed = gamepad1.a;

            if (gamepad1.b && !bWasPressed) {
                // reported X = -2*dy  and  reported Y = -2*dx, so the corrections are just
                // half the reported drift, subtracted from the offsets currently in use.
                suggestedX = RobotConstants.Odo.X_POD_OFFSET_MM - yMm / 2.0;
                suggestedY = RobotConstants.Odo.Y_POD_OFFSET_MM - xMm / 2.0;
                capturedHeadingDeg = headingDeg;
                capturedXmm = xMm;
                capturedYmm = yMm;
                captured = true;
            }
            bWasPressed = gamepad1.b;

            telemetry.addLine("=== ROTATE TO EXACTLY +180.0 DEG ===");
            telemetry.addData("Heading (deg)", "%+8.2f   target +180.00", headingDeg);
            telemetry.addData("Error from 180", "%+8.2f deg", 180.0 - Math.abs(headingDeg));
            telemetry.addLine();
            telemetry.addData("Reported X drift", "%+8.2f mm", xMm);
            telemetry.addData("Reported Y drift", "%+8.2f mm", yMm);
            telemetry.addData("Worst drift so far", "%8.2f mm", maxDriftMm);
            telemetry.addLine();
            telemetry.addLine("A = zero      B = capture and compute");
            telemetry.addLine();
            telemetry.addData("Offsets in use", "X %.2f   Y %.2f mm",
                    RobotConstants.Odo.X_POD_OFFSET_MM, RobotConstants.Odo.Y_POD_OFFSET_MM);

            if (captured) {
                telemetry.addLine();
                telemetry.addLine("=== RESULT (captured at heading "
                        + String.format("%+.2f", capturedHeadingDeg) + " deg) ===");
                telemetry.addData("drift was", "X %+.2f  Y %+.2f mm", capturedXmm, capturedYmm);
                telemetry.addLine();
                telemetry.addLine("Put these in RobotConstants.Odo:");
                telemetry.addData(" X_POD_OFFSET_MM", "%.2f", suggestedX);
                telemetry.addData(" Y_POD_OFFSET_MM", "%.2f", suggestedY);
                if (Math.abs(180.0 - Math.abs(capturedHeadingDeg)) > 2.0) {
                    telemetry.addLine();
                    telemetry.addLine("WARNING: captured more than 2 deg away from 180.");
                    telemetry.addLine("The result is unreliable. Zero and try again.");
                }
            }
            telemetry.update();
        }
    }
}
