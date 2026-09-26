package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * Drivetrain plus odometry, nothing else. <b>This is the OpMode to run while the turret and shooter
 * are still being built</b> -- it needs only four drive motors and the Pinpoint, so it will run on
 * the chassis as it exists today.
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li>Left stick: translate (field-centric by default)</li>
 *   <li>Right stick X: rotate</li>
 *   <li>Right trigger: slow mode</li>
 *   <li>Back: re-zero the field heading -- point the robot the way you want "field forward" to mean
 *       and press it</li>
 *   <li>Y: toggle field-centric / robot-centric, so your drivers can feel the difference</li>
 * </ul>
 *
 * <h2>Use this to prove odometry survives driving</h2>
 * Pushing the robot by hand in T1 is a gentle test. Real driving adds wheel scrub, impacts and
 * acceleration. Drive a lap of the field hard, come back to where you started, and see what the pose
 * reads. Under an inch or two of error after a vigorous lap is healthy. If the error grows every lap
 * in the same direction, suspect a pod that is not held firmly against the floor.
 */
@TeleOp(name = "TeleOp Drive Only (odometry)", group = "Drive")
public class TeleOpDriveOnly extends LinearOpMode {

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Hold the robot still. Press START.");
        telemetry.update();
        waitForStart();

        boolean fieldCentric = true;
        boolean yWas = false, backWas = false;

        double startX = odometry.getX(), startY = odometry.getY();

        while (opModeIsActive()) {
            odometry.update();

            if (gamepad1.y && !yWas) fieldCentric = !fieldCentric;
            yWas = gamepad1.y;

            if (gamepad1.back && !backWas) {
                odometry.resetPoseAndCalibrateImu();
                startX = 0;
                startY = 0;
            }
            backWas = gamepad1.back;

            // Negate left_stick_y because the gamepad reports "up" as negative.
            double forward = MecanumDrive.shapeStick(-gamepad1.left_stick_y);
            double left    = MecanumDrive.shapeStick(-gamepad1.left_stick_x);
            double yaw     = MecanumDrive.shapeStick(-gamepad1.right_stick_x);

            double scale = 1.0 - gamepad1.right_trigger * (1.0 - RobotConstants.Drive.SLOW_MODE_SCALE);
            forward *= scale;
            left *= scale;
            yaw *= scale;

            if (fieldCentric) {
                drive.driveFieldCentric(forward, left, yaw, odometry.getHeadingRad());
            } else {
                drive.driveRobotCentric(forward, left, yaw);
            }

            telemetry.addData("Mode", fieldCentric ? "FIELD-CENTRIC" : "robot-centric");
            telemetry.addData("Pinpoint", odometry.getStatusText());
            telemetry.addLine();
            telemetry.addData("X / Y (in)", "%+7.2f  %+7.2f", odometry.getX(), odometry.getY());
            telemetry.addData("Heading (deg)", "%+7.2f", odometry.getHeadingDeg());
            telemetry.addLine();
            telemetry.addData("Speed (in/s)", "%6.1f", odometry.getSpeed());
            telemetry.addData("Field vel X/Y", "%+6.1f  %+6.1f",
                    odometry.getFieldVelX(), odometry.getFieldVelY());
            telemetry.addData("Rotation (deg/s)", "%+6.1f", Math.toDegrees(odometry.getHeadingVelRad()));
            telemetry.addLine();
            telemetry.addData("Drift from start", "%.2f in",
                    Math.hypot(odometry.getX() - startX, odometry.getY() - startY));
            telemetry.addLine();
            telemetry.addLine("Y = toggle drive mode   BACK = re-zero heading");
            telemetry.addLine("Right trigger = slow mode");
            telemetry.update();
        }

        drive.stop();
    }
}
