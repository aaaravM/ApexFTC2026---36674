package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Bench test: intake + transfer + flywheel + kicker.
 * Gamepad 1 = same as T7. Gamepad 2 = kicker (hold Cross to kick, D-pad tunes positions).
 * The kicker does NOT move until gamepad 2 presses a button.
 */
@TeleOp(name = "T12 Shooter System Test", group = "Test")
public class T12_ShooterSystemTest extends OpMode {

  private static final double TICKS_PER_REV = RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV; // 28
  private static final double SERVO_POWER = 1.0;
  private static final double REVERSE_FLYWHEEL_POWER = 0.3;
  private static final double RPM_STEP = 100;
  private static final double KICK_STEP = 0.01;

  private DcMotorEx intakeMain, transfer, flywheel;
  private CRServo leftServo, rightServo;
  private Servo kicker;

  private double intakePower = 0.8;
  private double transferSpeed = 1.0;
  private double targetRpm = 3200;

  private double kickerRest = 0.50;
  private double kickerKick = 0.50;
  private boolean kickerEnabled = false;

  private boolean reachedSpeed = false;
  private double lowestRpm = Double.MAX_VALUE;

  // gamepad 1 edges
  private boolean prevUp, prevDown, prevLeft, prevRight, prevRB, prevLB;
  // gamepad 2 edges
  private boolean prev2Up, prev2Down, prev2Left, prev2Right;

  @Override
  public void init() {
    intakeMain = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    leftServo  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);
    rightServo = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);
    rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

    transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FEEDER);
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    flywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
    flywheel.setDirection(RobotConstants.Shoot.REVERSE_FLYWHEEL
        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setVelocityPIDFCoefficients(
        RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
        RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);

    kicker = hardwareMap.get(Servo.class, RobotConstants.Names.KICKER);
    // No setPosition() here: the kicker stays limp until gamepad 2 presses something.

    telemetry.addLine("GP1 = intake/transfer/flywheel (like T7). GP2 = kicker (hold Cross).");
    telemetry.update();
  }

  @Override
  public void loop() {
    // ===================== GAMEPAD 1 (same as T7) =====================
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

    // Intake
    double ip = 0, sp = 0;
    String intakeState = "stopped";
    if (gamepad1.a)      { ip =  intakePower; sp =  SERVO_POWER; intakeState = "IN"; }
    else if (gamepad1.b) { ip = -intakePower; sp = -SERVO_POWER; intakeState = "OUT"; }
    intakeMain.setPower(ip);
    leftServo.setPower(sp);
    rightServo.setPower(sp);

    // Transfer
    double tp = 0;
    String transferState = "holding";
    if (gamepad1.x)      { tp =  transferSpeed; transferState = "UP"; }
    else if (gamepad1.y) { tp = -transferSpeed; transferState = "BACK"; }
    transfer.setPower(tp);

    // Flywheel
    double rpm = flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    boolean ready = false;
    String shooterState;
    if (gamepad1.right_trigger > 0.5) {
      flywheel.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
      ready = Math.abs(rpm - targetRpm) <= RobotConstants.Shoot.RPM_TOLERANCE;
      if (ready) reachedSpeed = true;
      if (reachedSpeed) lowestRpm = Math.min(lowestRpm, rpm);
      shooterState = ready ? "READY" : "spinning up...";
    } else if (gamepad1.left_trigger > 0.5) {
      flywheel.setPower(-REVERSE_FLYWHEEL_POWER);
      shooterState = "REVERSE (clearing)";
      reachedSpeed = false;
    } else {
      flywheel.setPower(0);
      shooterState = "off";
      reachedSpeed = false;
    }
    if (gamepad1.back) lowestRpm = Double.MAX_VALUE;

    // ===================== GAMEPAD 2: KICKER =====================
    if (gamepad2.a || gamepad2.dpad_up || gamepad2.dpad_down
        || gamepad2.dpad_left || gamepad2.dpad_right) kickerEnabled = true;

    if (gamepad2.dpad_up && !prev2Up)       kickerKick = Math.min(1.0, kickerKick + KICK_STEP);
    if (gamepad2.dpad_down && !prev2Down)   kickerKick = Math.max(0.0, kickerKick - KICK_STEP);
    if (gamepad2.dpad_right && !prev2Right) kickerRest = Math.min(1.0, kickerRest + KICK_STEP);
    if (gamepad2.dpad_left && !prev2Left)   kickerRest = Math.max(0.0, kickerRest - KICK_STEP);
    prev2Up = gamepad2.dpad_up;       prev2Down = gamepad2.dpad_down;
    prev2Right = gamepad2.dpad_right; prev2Left = gamepad2.dpad_left;

    boolean kicking = gamepad2.a;
    if (kickerEnabled) kicker.setPosition(kicking ? kickerKick : kickerRest);

    // ===================== TELEMETRY =====================
    telemetry.addData("SHOOTER", shooterState);
    telemetry.addData("RPM (actual / target)", "%.0f / %.0f", rpm, targetRpm);
    telemetry.addData("Lowest RPM after spin-up",
        lowestRpm == Double.MAX_VALUE ? "-" : String.format("%.0f", lowestRpm));
    telemetry.addLine();
    telemetry.addData("KICKER", !kickerEnabled ? "not moving (GP2 press to enable)"
        : (kicking ? "KICK" : "rest") + (kicking && !ready ? "  (flywheel NOT ready!)" : ""));
    telemetry.addData("Kicker REST (GP2 D-pad L/R)", "%.2f", kickerRest);
    telemetry.addData("Kicker KICK (GP2 D-pad Up/Dn)", "%.2f", kickerKick);
    telemetry.addLine();
    telemetry.addData("Intake", "%s @ %.0f%%", intakeState, intakePower * 100);
    telemetry.addData("Transfer", "%s @ %.0f%%", transferState, transferSpeed * 100);
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