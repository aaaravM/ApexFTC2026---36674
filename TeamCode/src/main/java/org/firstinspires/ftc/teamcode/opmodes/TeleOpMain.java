package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AimController;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallSorter;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;

/**
 * The full robot: field-centric drive, auto-aiming turret that compensates for our own motion, and
 * a shooter whose speed depends on which ball is loaded.
 *
 * <p><b>Currently {@code @Disabled}.</b> It needs the turret, shooter, feeder, intake and colour
 * sensor all present in the robot configuration, and it will throw on init until they are. Remove
 * the {@code @Disabled} line once the hardware exists. Until then use {@link TeleOpDriveOnly}.
 *
 * <h2>Controls -- driver (gamepad 1)</h2>
 * <ul>
 *   <li>Left stick / right stick X: field-centric drive</li>
 *   <li>Right trigger: slow mode</li>
 *   <li>Back: re-zero field heading</li>
 * </ul>
 *
 * <h2>Controls -- operator (gamepad 2)</h2>
 * <ul>
 *   <li>Left bumper (hold): auto-aim on. Turret tracks the CELL, flywheel spins to the computed RPM.</li>
 *   <li>Right trigger: fire. Refuses unless all four readiness gates pass.</li>
 *   <li>A: intake in. B: intake out (also used to spit a rejected ball).</li>
 *   <li>X: clear the target estimate -- do this after the HIVE tips.</li>
 * </ul>
 *
 * <h2>The one habit worth building</h2>
 * The driver should hold auto-aim on for the whole scoring run rather than tapping it. The flywheel
 * takes a moment to settle, the turret takes a moment to slew, and the target estimate gets better
 * the longer the camera has been looking at the CELL. Holding it means every one of those is already
 * done when the driver decides to shoot.
 */
@TeleOp(name = "TeleOp Main (full robot)", group = "Drive")
@Disabled
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Set the alliance before the match. Easiest is to duplicate this OpMode into a red and a
        // blue version that each set this line, so nobody has to remember a button press.
        RobotConstants.alliance = RobotConstants.Alliance.BLUE;

        Odometry odometry = new Odometry(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        BallSorter ballSorter = new BallSorter(hardwareMap);
        HiveTracker hiveTracker = new HiveTracker(hardwareMap);
        AimController aim = new AimController(odometry, hiveTracker, turret, shooter, ballSorter);

        DcMotor intake = hardwareMap.get(DcMotor.class, RobotConstants.Names.INTAKE);
        CRServo feeder = hardwareMap.get(CRServo.class, RobotConstants.Names.FEEDER);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Setting camera exposure...");
        telemetry.update();
        hiveTracker.setManualExposure();

        // Zero the turret with it physically centred.
        turret.zeroHere();

        telemetry.addLine("Ready. Hold the robot still, then press START.");
        telemetry.addData("Alliance", RobotConstants.alliance);
        telemetry.update();
        waitForStart();

        boolean backWas = false, xWas = false;
        double loopTimeMs = 0;
        long lastLoopNano = System.nanoTime();

        while (opModeIsActive()) {
            // --- sensors, once each, in dependency order ---------------------------------------
            odometry.update();
            ballSorter.update();
            hiveTracker.update(odometry, turret.getAngleRad());

            // --- driver ------------------------------------------------------------------------
            if (gamepad1.back && !backWas) odometry.resetPoseAndCalibrateImu();
            backWas = gamepad1.back;

            double forward = MecanumDrive.shapeStick(-gamepad1.left_stick_y);
            double left    = MecanumDrive.shapeStick(-gamepad1.left_stick_x);
            double yaw     = MecanumDrive.shapeStick(-gamepad1.right_stick_x);
            double scale = 1.0 - gamepad1.right_trigger * (1.0 - RobotConstants.Drive.SLOW_MODE_SCALE);
            drive.driveFieldCentric(forward * scale, left * scale, yaw * scale,
                    odometry.getHeadingRad());

            // --- operator ---------------------------------------------------------------------
            if (gamepad2.x && !xWas) hiveTracker.resetEstimate();
            xWas = gamepad2.x;

            aim.setAimingEnabled(gamepad2.left_bumper);
            aim.update();

            // Intake, with automatic rejection of the opponent's NECTAR.
            if (ballSorter.shouldReject()) {
                intake.setPower(-1.0);
                feeder.setPower(-1.0);
            } else if (gamepad2.a) {
                intake.setPower(1.0);
                feeder.setPower(0);
            } else if (gamepad2.b) {
                intake.setPower(-1.0);
                feeder.setPower(-1.0);
            } else {
                intake.setPower(0);
                // Fire only when every gate is satisfied. The trigger is a request, not a command.
                feeder.setPower(gamepad2.right_trigger > 0.5 && aim.isReadyToFire() ? 1.0 : 0.0);
            }

            // --- telemetry --------------------------------------------------------------------
            long now = System.nanoTime();
            loopTimeMs = (now - lastLoopNano) / 1e6;
            lastLoopNano = now;

            telemetry.addLine(">>> " + aim.getReadinessText() + " <<<");
            telemetry.addLine();
            telemetry.addData("Ball loaded", "%s%s", ballSorter.getBallType(),
                    ballSorter.shouldReject() ? "  *** EJECTING (wrong alliance) ***" : "");
            telemetry.addData("Firing as", aim.getAssumedBall());
            telemetry.addLine();
            telemetry.addData("Shooter", "%.0f / %.0f rpm",
                    shooter.getCurrentRpm(), shooter.getTargetRpm());
            telemetry.addData("Turret", "%+.1f -> %+.1f deg  (err %+.1f)",
                    turret.getAngleDeg(), turret.getTargetAngleDeg(), turret.getErrorDeg());
            telemetry.addLine();
            telemetry.addData("Target", hiveTracker.hasTarget()
                    ? String.format("%s  %s  age %.2fs", hiveTracker.getLastClusterName(),
                        hiveTracker.isSightingLive() ? "LIVE" : "dead-reckoned",
                        hiveTracker.getEstimateAgeSec())
                    : "NONE");
            telemetry.addData("Solution", aim.getSolution().toString());
            telemetry.addLine();
            telemetry.addData("Pose", "X %+6.1f  Y %+6.1f  H %+6.1f",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
            telemetry.addData("Speed", "%.1f in/s", odometry.getSpeed());
            telemetry.addLine();
            telemetry.addData("Loop", "%.1f ms   Pinpoint %s", loopTimeMs, odometry.getStatusText());
            telemetry.update();
        }

        drive.stop();
        turret.stop();
        shooter.stop();
        intake.setPower(0);
        feeder.setPower(0);
        hiveTracker.close();
    }
}
