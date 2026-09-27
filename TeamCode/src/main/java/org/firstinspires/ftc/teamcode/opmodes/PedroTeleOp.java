package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.PedroOdometry;

/**
 * The same robot as {@link MecanumTeleOpOdometry}, but driven through Pedro Pathing.
 *
 * <h2>This is where "connect it to the odometry" and "make it rotate" become one fix</h2>
 * Pedro's {@code Mecanum} drivetrain does the four-wheel mixing itself --
 * {@code fl = f-s-t; fr = f+s+t; bl = f+s-t; br = f-s+t}, byte-for-byte the mixing already in your
 * teleop -- and the third argument of {@code follower.manual()} is the turn term. So routing the
 * gamepad through Pedro gives you rotation with no mecanum maths of your own, and pose and velocity
 * come out of the same object for free.
 *
 * <p>It also means Pedro becomes the <b>only</b> owner of the four drive motors and the Pinpoint.
 * Nothing else may touch them -- see {@link PedroOdometry} for what goes wrong if you also construct
 * the standalone {@code Odometry}.
 *
 * <h2>Two calls, in this order, every loop</h2>
 * <pre>
 *   follower.manual(forward, lateral, turn);   // only STORES the request
 *   follower.update();                         // reads the localizer AND writes motor powers
 * </pre>
 * {@code manual()} on its own does nothing to the motors. {@code update()} must be called exactly
 * once per loop, after the manual call, and <b>before</b> you read pose or velocity for telemetry or
 * for the turret -- read it before and you are using last loop's numbers.
 *
 * <h2>Field-centric is your choice at the call site</h2>
 * {@code follower.manual(f, l, t)} is robot-centric. Pedro does not apply the heading rotation for
 * you and there is no flag for it: you call {@code ManualDrive.fieldCentric(f, l, t, heading)} and
 * hand the resulting {@link DrivePowers} to {@code manual()}. Both paths are wired below so your
 * drivers can feel the difference (press Y).
 *
 * <h2>Controls -- unchanged from your existing teleop</h2>
 * <ul>
 *   <li>Left stick: translate. Right stick X: <b>rotate</b>.</li>
 *   <li>Right trigger: intake in. Left trigger: intake out.</li>
 *   <li>Right bumper: slow mode.</li>
 *   <li>Y: toggle field-centric / robot-centric. BACK: reset pose to (0,0,0).</li>
 * </ul>
 *
 * <p><b>If Pedro drives but will not turn</b>, check the four motor names in
 * {@link PedroConstants} against the physical robot first. Pedro renamed v2's
 * {@code leftRearMotorName} to {@code backLeftName} -- word order flipped and "rear" became "back" --
 * and two transposed motors drive almost correctly in translation while being completely wrong in
 * rotation. That reads as a rotation bug but is a naming bug.
 */
@TeleOp(name = "Pedro TeleOp", group = "Drive")
public class PedroTeleOp extends LinearOpMode {

    private Follower follower;
    private PedroOdometry odometry;

    private DcMotorEx intakeMain;
    private CRServo rightIntake;
    private CRServo leftIntake;

    private static final double STICK_DEADBAND = 0.05;
    private static final double TRIGGER_DEADBAND = 0.10;

    @Override
    public void runOpMode() {
        follower = PedroConstants.createFollower(hardwareMap);
        odometry = new PedroOdometry(hardwareMap, follower);

        // Intake and outtake have nothing to do with Pedro -- it only owns the drive motors and the
        // Pinpoint. These stay plain SDK calls.
        intakeMain  = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
        rightIntake = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);
        leftIntake  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);

        intakeMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        // Start the pose at the origin. In autonomous, set this to your real starting position.
        follower.setPose(new Pose(0, 0, 0));

        // The line that makes telemetry live. The SDK default is 250 ms -- 4 Hz -- and it silently
        // drops any update superseded before the interval expires.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addLine("Pedro Pathing follower built.");
        telemetry.addLine("HOLD THE ROBOT STILL -- the Pinpoint IMU is calibrating.");
        telemetry.addData("Pinpoint", odometry.getStatusText());
        telemetry.update();

        waitForStart();

        boolean fieldCentric = true;
        boolean yWas = false, backWas = false;
        int loops = 0;
        long lastNano = System.nanoTime();
        double loopMs = 0;

        while (opModeIsActive()) {
            loops++;
            long now = System.nanoTime();
            loopMs = (now - lastNano) / 1e6;
            lastNano = now;

            if (gamepad1.y && !yWas) fieldCentric = !fieldCentric;
            yWas = gamepad1.y;

            if (gamepad1.back && !backWas) follower.setPose(new Pose(0, 0, 0));
            backWas = gamepad1.back;

            // Negate left_stick_y: the gamepad reports "up" as negative.
            // Pedro's convention is +forward, +lateral = LEFT, +turn = CCW -- the same convention
            // this codebase already uses everywhere, so no sign juggling is needed.
            double forward = deadband(-gamepad1.left_stick_y);
            double lateral = deadband(-gamepad1.left_stick_x);
            double turn    = deadband(-gamepad1.right_stick_x);

            double scale = gamepad1.right_bumper ? RobotConstants.Drive.SLOW_MODE_SCALE : 1.0;
            forward *= scale;
            lateral *= scale;
            turn *= scale;

            if (fieldCentric) {
                DrivePowers powers = ManualDrive.fieldCentric(
                        forward, lateral, turn, follower.pose().heading());
                follower.manual(powers);
            } else {
                follower.manual(forward, lateral, turn);
            }

            // This is the call that actually moves the robot and refreshes the localizer.
            follower.update();

            // Sample pose/velocity only after update().
            odometry.update();

            // --- intake ---------------------------------------------------------------------
            // Motor and servos share ONE threshold. The original used a bare
            // (right_trigger - left_trigger) for the motor while the servos used 0.1, so a trigger
            // resting slightly off zero spun the motor with the servos idle -- and holding both
            // triggers stopped the motor dead while the servos kept feeding.
            double intakeIn  = gamepad1.right_trigger > TRIGGER_DEADBAND ? gamepad1.right_trigger : 0.0;
            double intakeOut = gamepad1.left_trigger  > TRIGGER_DEADBAND ? gamepad1.left_trigger  : 0.0;
            double intakePower = intakeIn - intakeOut;
            intakeMain.setPower(intakePower);

            // Drive the servos from the same signed value, so they can never fight the motor.
            double servoPower = Math.signum(intakePower);
            leftIntake.setPower(servoPower);
            rightIntake.setPower(servoPower);

            // --- telemetry ------------------------------------------------------------------
            telemetry.addData("Mode", fieldCentric ? "FIELD-CENTRIC" : "robot-centric");
            telemetry.addData("Follower", follower.mode());
            telemetry.addData("Pinpoint", odometry.getStatusText());

            telemetry.addLine();
            telemetry.addLine("=== POSE (Pedro) ===");
            telemetry.addData("X / Y (in)", "%+8.2f  %+8.2f", odometry.getX(), odometry.getY());
            telemetry.addData("Heading (deg)", "%+8.2f", odometry.getHeadingDeg());

            telemetry.addLine();
            telemetry.addLine("=== VELOCITY ===");
            telemetry.addData("field vx/vy", "%+7.1f  %+7.1f in/s",
                    odometry.getFieldVelX(), odometry.getFieldVelY());
            telemetry.addData("robot fwd/left", "%+7.1f  %+7.1f in/s",
                    odometry.getRobotVelForward(), odometry.getRobotVelLeft());
            telemetry.addData("speed", "%7.1f in/s", odometry.getSpeed());
            telemetry.addData("omega", "%+7.1f deg/s", Math.toDegrees(odometry.getHeadingVelRad()));

            telemetry.addLine();
            telemetry.addLine("=== INPUTS ===");
            telemetry.addData("right_stick_x", "%+6.3f  <- rotation", gamepad1.right_stick_x);
            telemetry.addData("fwd/lat/turn", "%+5.2f  %+5.2f  %+5.2f", forward, lateral, turn);
            telemetry.addData("intake", "%+5.2f", intakePower);

            telemetry.addLine();
            telemetry.addData("loop", "#%d  %.1f ms  (climbing = data is live)", loops, loopMs);
            telemetry.addLine("Y = drive mode   BACK = reset pose   RB = slow");
            telemetry.update();
        }

        follower.stop();
        intakeMain.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    private static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }
}
