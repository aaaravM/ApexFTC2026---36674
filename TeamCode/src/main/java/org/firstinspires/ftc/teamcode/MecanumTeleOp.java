package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The team's original teleop, preserved as written so it is under version control and available on
 * every computer. Unmodified except for this comment and formatting.
 *
 * <p>For driving and debugging, prefer {@link MecanumTeleOpOdometry} -- it is this same code plus
 * odometry, live telemetry and a rotation diagnostic. This file is kept as the known-good fallback:
 * if a change ever breaks the new one at a competition, this still drives.
 */
@TeleOp(name = "MecanumTeleOp (original)", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Drive Motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    // Intake Motor
    private DcMotor intakeMain = null;

    // Intake Servos
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftfront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightback");
        leftBack   = hardwareMap.get(DcMotor.class, "leftback");

        intakeMain  = hardwareMap.get(DcMotor.class, "intakemain");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");
        leftIntake  = hardwareMap.get(CRServo.class, "leftintake");

        // Drive motor directions. Changed from hardcoded to reading RobotConstants.Drive, so this
        // fallback OpMode cannot drift out of sync with the one you normally drive. After the
        // 2026-10-11 rewire all four were inverted.
        leftFront.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_LEFT));
        leftBack.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_LEFT));
        rightFront.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_RIGHT));
        rightBack.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_RIGHT));

        // Servo directions
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        // Brake mode on drivetrain
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- DRIVE CODE ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            // --- INTAKE & SERVO CODE ---
            // Variable motor power based on trigger depth
            double motorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            intakeMain.setPower(motorPower);

            // Maximum speed (100%) for servos whenever trigger is pressed
            if (gamepad1.right_trigger > 0.1) {
                leftIntake.setPower(1.0);
                rightIntake.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                leftIntake.setPower(-1.0);
                rightIntake.setPower(-1.0);
            } else {
                leftIntake.setPower(0.0);
                rightIntake.setPower(0.0);
            }

            telemetry.addData("Servo Power", leftIntake.getPower());
            telemetry.update();
        }
    }

    /** Boolean "is reversed" from RobotConstants to the SDK's Direction enum. */
    private static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
