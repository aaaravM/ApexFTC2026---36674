package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Bench test for intake, transfer and flywheel together.
 * Self-contained: drives the hardware directly using RobotConstants names.
 */
@TeleOp(name = "Intake + Transfer + Shooter Test", group = "Test")
public class T7_IntakeTransferShooterTest extends OpMode {

  private static final double TICKS_PER_REV = RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV; // 28
  private static final double SERVO_POWER = 1.0;
  private static final double REVERSE_FLYWHEEL_POWER = 0.3;  // slow, for clearing jams
  private static final double RPM_STEP = 100;

  private DcMotorEx intakeMain, transfer, flywheel;
  private CRServo leftServo, rightServo;

  private double intakePower = 0.8;
  private double transferSpeed = 1.0;
  private double targetRpm = 3200;   // starting guess; tune with the bumpers

  private boolean prevUp, prevDown, prevLeft, prevRight, prevRB, prevLB;

  private boolean reachedSpeed = false;          // has it hit target since spin-up?
  private double lowestRpm = Double.MAX_VALUE;   // lowest RPM after first reaching speed

  @Override
  public void init() {
    // ---- Intake (from the Intake + Transfer test on the rewired robot) ----
    intakeMain = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    leftServo  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);
    rightServo = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);
    rightServo.setDirection(DcMotorSimple.Direction.REVERSE); // mirrored servo

    // ---- Transfer ----
    transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FEEDER);
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // holds balls back

    // ---- Flywheel: velocity control using the encoder ----
    flywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
    flywheel.setDirection(RobotConstants.Shoot.REVERSE_FLYWHEEL
        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setVelocityPIDFCoefficients(
        RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
        RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);
  }

  @Override
  public void loop() {
    // ---- Adjustments (each press counts once) ----
    if (gamepad1.dpad_up && !prevUp)       intakePower   = Math.min(1.0, intakePower + 0.1);
    if (gamepad1.dpad_down && !prevDown)   intakePower   = Math.max(0.1, intakePower - 0.1);
    if (gamepad1.dpad_right && !prevRight) transferSpeed = Math.min(1.0, transferSpeed + 0.1);
    if (gamepad1.dpad_left && !prevLeft)   transferSpeed = Math.max(0.1, transferSpeed - 0.1);
    if (gamepad1.right_bumper && !prevRB)
      targetRpm = Math.min(RobotConstants.Shoot.MAX_FLYWHEEL_RPM, targetRpm + RPM_STEP);
    if (gamepad1.left_bumper && !prevLB)
      targetRpm = Math.max(0, targetRpm - RPM_STEP);
    prevUp = gamepad1.dpad_up;       prevDown = gamepad1.dpad_down;
    prevRight = gamepad1.dpad_right; prevLeft = gamepad1.dpad_left;
    prevRB = gamepad1.right_bumper;  prevLB = gamepad1.left_bumper;

    // ---- Intake: Cross = in, Circle = out ----
    double ip = 0, sp = 0;
    String intakeState = "stopped";
    if (gamepad1.a)      { ip =  intakePower; sp =  SERVO_POWER; intakeState = "IN"; }
    else if (gamepad1.b) { ip = -intakePower; sp = -SERVO_POWER; intakeState = "OUT"; }
    intakeMain.setPower(ip);
    leftServo.setPower(sp);
    rightServo.setPower(sp);

    // ---- Transfer: Square = up to flywheel, Triangle = back ----
    double tp = 0;
    String transferState = "holding";
    if (gamepad1.x)      { tp =  transferSpeed; transferState = "UP"; }
    else if (gamepad1.y) { tp = -transferSpeed; transferState = "BACK"; }
    transfer.setPower(tp);

    // ---- Flywheel: right trigger = spin to target, left trigger = slow reverse ----
    double rpm = flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    boolean spinning = gamepad1.right_trigger > 0.5;
    String shooterState;

    if (spinning) {
      flywheel.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
      boolean atSpeed = Math.abs(rpm - targetRpm) <= RobotConstants.Shoot.RPM_TOLERANCE;
      if (atSpeed) reachedSpeed = true;
      if (reachedSpeed) lowestRpm = Math.min(lowestRpm, rpm);
      shooterState = atSpeed ? "READY" : "spinning up...";
    } else if (gamepad1.left_trigger > 0.5) {
      flywheel.setPower(-REVERSE_FLYWHEEL_POWER);
      shooterState = "REVERSE (clearing)";
      reachedSpeed = false;
    } else {
      flywheel.setPower(0);
      shooterState = "off";
      reachedSpeed = false;
    }

    if (gamepad1.back) lowestRpm = Double.MAX_VALUE;   // reset dip reading

    telemetry.addData("SHOOTER", shooterState);
    telemetry.addData("Target RPM (bumpers)", "%.0f", targetRpm);
    telemetry.addData("Actual RPM", "%.0f", rpm);
    telemetry.addData("Error", "%+.0f", rpm - targetRpm);
    telemetry.addData("Lowest RPM after spin-up (Back resets)",
        lowestRpm == Double.MAX_VALUE ? "-" : String.format("%.0f", lowestRpm));
    telemetry.addLine();
    telemetry.addData("Intake (Cross/Circle)", "%s @ %.0f%%", intakeState, intakePower * 100);
    telemetry.addData("Transfer (Square/Triangle)", "%s @ %.0f%%", transferState, transferSpeed * 100);
    telemetry.update();
  }

  @Override
  public void stop() {
    intakeMain.setPower(0);
    leftServo.setPower(0);
    rightServo.setPower(0);
    transfer.setPower(0);
    flywheel.setPower(0);
  }
}