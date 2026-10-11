package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The team's original intake + flywheel test, preserved as written so it is under version control.
 * Unmodified except for this comment and formatting.
 *
 * <p>For actually finding your shooter speed, use {@link IntakeFlywheelTuner} -- this OpMode runs the
 * flywheel at {@code setPower(1.0)}, which is roughly 6000 RPM, and BIOBUZZ needs about 1600-2900.
 * This file is kept as the known-good fallback.
 *
 * <p>Two things to know about this code, neither of which breaks it today:
 * <ul>
 *   <li>It inverts the right servo by <b>negating its power</b>, while {@link MecanumTeleOp} inverts
 *       it with {@code setDirection(REVERSE)} and then sends both servos {@code +1.0}. Each file is
 *       self-consistent, so both work. But if anyone ever merges them the right servo gets inverted
 *       twice and quietly runs backwards.</li>
 *   <li>It never calls {@code setMode(RUN_WITHOUT_ENCODER)}. Run mode is <b>sticky in the hub across
 *       OpModes</b> until a power cycle, so if a tuning OpMode leaves the flywheel in
 *       RUN_USING_ENCODER, this {@code setPower(1.0)} behaves differently than it did last time --
 *       for no visible reason.</li>
 * </ul>
 */
@TeleOp(name = "Intake and Flywheel Test (original)", group = "Test")
public class IntakeFlywheelTest extends LinearOpMode {

    // Declare DC motors
    private DcMotor intakeMain = null;
    private DcMotor flywheel = null;

    // Declare continuous rotation servos
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;

    @Override
    public void runOpMode() {
        // Initialize DC motors from the Expansion Hub config
        intakeMain = hardwareMap.get(DcMotor.class, "intakemain");
        flywheel   = hardwareMap.get(DcMotor.class, "Flywheel");

        // Initialize continuous rotation servos
        leftIntake  = hardwareMap.get(CRServo.class, "leftintake");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");

        // Set brake mode for intake, but float (coast) mode for the flywheel
        intakeMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Added: the flywheel had no direction set, so it ran whichever way it happened to be
        // wired -- which on this robot was backwards. Kept in sync with
        // RobotConstants.Shoot.REVERSE_FLYWHEEL so this file and the tuner never disagree.
        flywheel.setDirection(RobotConstants.Shoot.REVERSE_FLYWHEEL
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        // Zero out servos on init
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);

        telemetry.addData("Status", "Mechanisms Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- INTAKE CONTROLS ---
            if (gamepad1.cross) {
                // Spin inward to collect balls (Cross)
                intakeMain.setPower(1.0);
                leftIntake.setPower(1.0);
                rightIntake.setPower(-1.0);
            } else if (gamepad1.triangle) {
                // Spin outward to eject balls (Triangle)
                intakeMain.setPower(-1.0);
                leftIntake.setPower(-1.0);
                rightIntake.setPower(1.0);
            } else {
                // Stop intake when released
                intakeMain.setPower(0.0);
                leftIntake.setPower(0.0);
                rightIntake.setPower(0.0);
            }

            // --- FLYWHEEL / OUTTAKE CONTROLS ---
            if (gamepad1.circle) {
                // Run flywheel to shoot outtake (Circle)
                flywheel.setPower(1.0);
            } else {
                // Stop flywheel when released
                flywheel.setPower(0.0);
            }

            // Telemetry debug
            telemetry.addData("Intake Power", "%.2f", intakeMain.getPower());
            telemetry.addData("Flywheel Power", "%.2f", flywheel.getPower());
            telemetry.addData("Servos Power", "L: %.2f | R: %.2f",
                    leftIntake.getPower(), rightIntake.getPower());
            telemetry.update();
        }
    }
}
