package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * STEP 3. Answers one question: does the Pinpoint report velocity in the FIELD frame or the ROBOT
 * frame? Takes about twenty seconds and you only ever do it once.
 *
 * <h2>Why this is not already known</h2>
 * goBILDA's driver documents the velocity getters as "X (forward) velocity" and "Y (strafe)
 * velocity", which sounds robot-relative -- but it documents <i>position</i> with the same words,
 * and position is unambiguously a field coordinate (it is the integral that the heading feeds into).
 * The labels come from the pod names, so they are not evidence. Measuring settles it.
 *
 * <h2>Why it matters enough to bother</h2>
 * The moving-shot solver needs field velocity to work out the lead. If the frame is wrong, the maths
 * still runs and still produces a confident-looking answer -- it is just wrong whenever the robot's
 * heading is not zero. From the driver's seat that shows up as a turret that leads correctly when
 * you drive one direction and leads backwards when you drive another. That is a miserable thing to
 * diagnose during a competition.
 *
 * <h2>The test</h2>
 * Point the robot along field +Y (heading +90 degrees) and push it straight forward.
 * <ul>
 *   <li>If velocity is <b>ROBOT-relative</b>, all the signal lands on velX (forward), velY stays 0.</li>
 *   <li>If velocity is <b>FIELD-relative</b>, all the signal lands on velY, velX stays 0.</li>
 * </ul>
 * At 90 degrees the two answers are maximally different, so there is nothing to interpret.
 *
 * <h2>Procedure</h2>
 * <ol>
 *   <li>Press START. Press <b>A</b> to zero the heading with the robot where it sits.</li>
 *   <li>Rotate the robot to heading <b>+90 degrees</b> (telemetry shows it live).</li>
 *   <li>Press <b>X</b> to start recording.</li>
 *   <li>Push the robot straight FORWARD (the way it is now facing) about three feet, briskly.</li>
 *   <li>Read the verdict and set {@code RobotConstants.Odo.VELOCITY_IS_ROBOT_RELATIVE} to match.</li>
 * </ol>
 */
@TeleOp(name = "T3 Velocity Frame Test", group = "Tuning")
public class T3_VelocityFrameTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        boolean recording = false;
        double peakVelX = 0, peakVelY = 0;
        boolean aWas = false, xWas = false, yWas = false;

        while (opModeIsActive()) {
            odometry.update();

            if (gamepad1.a && !aWas) odometry.resetPoseAndCalibrateImu();
            aWas = gamepad1.a;

            if (gamepad1.x && !xWas) {
                recording = true;
                peakVelX = 0;
                peakVelY = 0;
            }
            xWas = gamepad1.x;

            if (gamepad1.y && !yWas) recording = false;
            yWas = gamepad1.y;

            if (recording) {
                if (Math.abs(odometry.getRawVelX()) > Math.abs(peakVelX)) peakVelX = odometry.getRawVelX();
                if (Math.abs(odometry.getRawVelY()) > Math.abs(peakVelY)) peakVelY = odometry.getRawVelY();
            }

            telemetry.addData("Heading (deg)", "%+7.2f   <- get this to +90", odometry.getHeadingDeg());
            telemetry.addLine();
            telemetry.addData("Recording", recording ? "YES -- push it forward now" : "no");
            telemetry.addData("live velX / velY", "%+7.1f  %+7.1f",
                    odometry.getRawVelX(), odometry.getRawVelY());
            telemetry.addData("peak velX / velY", "%+7.1f  %+7.1f", peakVelX, peakVelY);
            telemetry.addLine();
            telemetry.addLine("A = zero heading   X = start recording   Y = stop");
            telemetry.addLine();

            double absX = Math.abs(peakVelX);
            double absY = Math.abs(peakVelY);
            double total = absX + absY;

            if (total < 5) {
                telemetry.addLine("VERDICT: not enough motion yet. Push harder.");
            } else if (Math.abs(odometry.getHeadingDeg() - 90) > 15 && !recording) {
                telemetry.addLine("VERDICT: heading was not near +90, result not meaningful.");
            } else if (absX > absY * 3) {
                telemetry.addLine("VERDICT: velocity is ROBOT-relative.");
                telemetry.addLine("  (all the motion showed up on velX = forward)");
                telemetry.addLine("  set VELOCITY_IS_ROBOT_RELATIVE = true");
            } else if (absY > absX * 3) {
                telemetry.addLine("VERDICT: velocity is FIELD-relative.");
                telemetry.addLine("  (the motion showed up on velY = field +Y)");
                telemetry.addLine("  set VELOCITY_IS_ROBOT_RELATIVE = false");
            } else {
                telemetry.addLine("VERDICT: unclear -- the split is too even.");
                telemetry.addLine("  Check the heading really was +90, and that you");
                telemetry.addLine("  pushed straight along the robot's forward axis.");
            }
            telemetry.update();
        }
    }
}
