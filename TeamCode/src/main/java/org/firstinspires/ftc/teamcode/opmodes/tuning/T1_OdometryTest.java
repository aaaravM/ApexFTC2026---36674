package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * STEP 1. Run this before writing or tuning anything else. No drivetrain needed -- push the robot
 * around by hand.
 *
 * <h2>What you are checking, in order</h2>
 * <ol>
 *   <li><b>Status says READY.</b> If it says a pod is not detected, it is a cable or a config name,
 *       not a number. Fix that first; nothing below will make sense.</li>
 *   <li><b>Frequency is around 1500 Hz</b> and the Pinpoint loop time is roughly 650 us. A much
 *       lower frequency means I2C trouble -- most often the Pinpoint is on I2C bus 0, which it
 *       shares with the Control Hub's internal IMU. Move it to bus 1, 2 or 3.</li>
 *   <li><b>Push the robot FORWARD: X must increase.</b> If it decreases, set
 *       {@code REVERSE_X_ENCODER = true}.</li>
 *   <li><b>Push the robot LEFT: Y must increase.</b> If it decreases, set
 *       {@code REVERSE_Y_ENCODER = true}.</li>
 *   <li><b>Rotate the robot counter-clockwise: heading must increase.</b> If it decreases the
 *       Pinpoint is mounted upside down -- it must sit sticker side up. This is a hardware fix, not
 *       a software one.</li>
 *   <li><b>Distance accuracy.</b> Push the robot exactly 48 inches along a tape measure. X should
 *       read 48 within about half an inch. If it is off by a consistent percentage, the pod type is
 *       wrong (we are set up for the 4-Bar pod at 19.894 ticks/mm; the Swingarm pod is 13.263) or
 *       a pod wheel is slipping.</li>
 * </ol>
 *
 * <p>Get all six right before touching pod offsets. Offsets only affect rotation behaviour, so
 * chasing them while an encoder is reversed wastes an afternoon.
 */
@TeleOp(name = "T1 Odometry Test", group = "Tuning")
public class T1_OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Pinpoint initialised. Hold the robot STILL.");
        telemetry.addLine("Press START, then push the robot around by hand.");
        telemetry.update();

        waitForStart();

        double maxSpeed = 0;

        while (opModeIsActive()) {
            odometry.update();
            maxSpeed = Math.max(maxSpeed, odometry.getSpeed());

            if (gamepad1.a) {
                odometry.resetPoseAndCalibrateImu();
                maxSpeed = 0;
            }

            telemetry.addLine("=== PINPOINT HEALTH ===");
            telemetry.addData("Status", odometry.getStatusText());
            telemetry.addData("Frequency", "%.0f Hz  (expect ~1500)", odometry.getPinpointFrequency());
            telemetry.addData("Loop time", "%d us  (expect ~650)", odometry.getPinpointLoopTimeUs());

            telemetry.addLine();
            telemetry.addLine("=== POSE ===");
            telemetry.addData("X (in)", "%+8.2f   <- push FORWARD, must go UP", odometry.getX());
            telemetry.addData("Y (in)", "%+8.2f   <- push LEFT, must go UP", odometry.getY());
            telemetry.addData("Heading (deg)", "%+8.2f   <- rotate CCW, must go UP",
                    odometry.getHeadingDeg());

            telemetry.addLine();
            telemetry.addLine("=== VELOCITY ===");
            telemetry.addData("Speed (in/s)", "%6.1f   peak %.1f", odometry.getSpeed(), maxSpeed);
            telemetry.addData("Raw velX/velY", "%+7.1f  %+7.1f",
                    odometry.getRawVelX(), odometry.getRawVelY());
            telemetry.addData("Rotation (deg/s)", "%+7.1f",
                    Math.toDegrees(odometry.getHeadingVelRad()));

            telemetry.addLine();
            telemetry.addLine("=== RAW ENCODERS ===");
            telemetry.addData("X pod ticks", odometry.getEncoderX());
            telemetry.addData("Y pod ticks", odometry.getEncoderY());
            telemetry.addLine("(If a tick count never changes, that pod's wheel is not");
            telemetry.addLine(" touching the floor or its cable is loose.)");

            telemetry.addLine();
            telemetry.addLine("A = reset pose and recalibrate IMU (hold robot still)");
            telemetry.update();
        }
    }
}
