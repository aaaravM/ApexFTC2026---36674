package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Match TeleOp for the robot as it is today: drivetrain, intake, transfer, flywheel.
 * No turret / camera / ball sorter (those belong to TeleOpMain).
 *
 * Gamepad 1 drives (robot-centric). Gamepad 2 runs the mechanisms.
 * Feeding is gated on flywheel speed, so each ball waits for the flywheel to recover.
 */
@TeleOp(name = "TeleOp Basic", group = "Match")
public class TeleOpBasic extends OpMode {

  // ---- Tunables ----
  private static final double TICKS_PER_REV = RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV; // 28
  private static final double INTAKE_POWER = 0.8;
  private static final double SERVO_POWER = 1.0;
  private static final double TRANSFER_POWER = 1.0;
  private static final double RPM_STEP = 100;

  // ---- Hardware ----
  private DcMotorEx frontLeft, frontRight, backLeft, backRight;
  private DcMotorEx intakeMain, transfer, flywheel;
  private CRServo leftServo, rightServo;

  // ---- State ----
  private boolean flywheelOn = false;
  private double targetRpm = 3200;   // adjust with gamepad 2 bumpers
  private boolean wasReady = false;
  private boolean prevR2, prevRB, prevLB;

  @Override
  public void init() {
    // ---- Drivetrain (directions verified with TeleOpDriveOnly) ----
    frontLeft  = initDrive(RobotConstants.Names.FRONT_LEFT,  RobotConstants.Drive.REVERSE_FRONT_LEFT);
    frontRight = initDrive(RobotConstants.Names.FRONT_RIGHT, RobotConstants.Drive.REVERSE_FRONT_RIGHT);
    backLeft   = initDrive(RobotConstants.Names.BACK_LEFT,   RobotConstants.Drive.REVERSE_BACK_LEFT);
    backRight  = initDrive(RobotConstants.Names.BACK_RIGHT,  RobotConstants.Drive.REVERSE_BACK_RIGHT);

    // ---- Intake ----
    intakeMain = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    leftServo  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);
    rightServo = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);
    rightServo.setDirection(DcMotorSimple.Direction.REVERSE); // mirrored servo

    // ---- Transfer ----
    transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FEEDER);
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // holds balls back

    // ---- Flywheel (velocity control) ----
    flywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
    flywheel.setDirection(RobotConstants.Shoot.REVERSE_FLYWHEEL
        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setVelocityPIDFCoefficients(
        RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
        RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);

    telemetry.addLine("Ready. GP1 drives, GP2 runs intake / transfer / shooter.");
    telemetry.update();
  }

  private DcMotorEx initDrive(String name, boolean reversed) {
    DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
    m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    return m;
  }

  private double deadband(double v) {
    return Math.abs(v) < RobotConstants.Drive.STICK_DEADBAND ? 0 : v;
  }

  @Override
  public void loop() {
    // ================= GAMEPAD 1: DRIVE (robot-centric) =================
    double axial   = deadband(-gamepad1.left_stick_y);  // forward is positive
    double lateral = deadband(gamepad1.left_stick_x);   // right is positive
    double yaw     = deadband(gamepad1.right_stick_x);  // clockwise is positive

    double scale = gamepad1.left_trigger > 0.5 ? RobotConstants.Drive.SLOW_MODE_SCALE : 1.0;

    double fl = axial + lateral + yaw;
    double fr = axial - lateral - yaw;
    double bl = axial - lateral + yaw;
    double br = axial + lateral - yaw;

    // Keep the ratios when any wheel would exceed full power
    double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
        Math.max(Math.abs(bl), Math.abs(br))));
    frontLeft.setPower(fl / max * scale);
    frontRight.setPower(fr / max * scale);
    backLeft.setPower(bl / max * scale);
    backRight.setPower(br / max * scale);

    // ================= GAMEPAD 2: MECHANISMS =================

    // ---- Flywheel toggle (R2) and RPM adjust (R1 / L1) ----
    boolean r2 = gamepad2.right_trigger > 0.5;
    if (r2 && !prevR2) flywheelOn = !flywheelOn;
    prevR2 = r2;

    if (gamepad2.right_bumper && !prevRB)
      targetRpm = Math.min(RobotConstants.Shoot.MAX_FLYWHEEL_RPM, targetRpm + RPM_STEP);
    if (gamepad2.left_bumper && !prevLB)
      targetRpm = Math.max(0, targetRpm - RPM_STEP);
    prevRB = gamepad2.right_bumper;
    prevLB = gamepad2.left_bumper;

    double rpm = flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    boolean ready = false;
    if (flywheelOn) {
      flywheel.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
      ready = Math.abs(rpm - targetRpm) <= RobotConstants.Shoot.RPM_TOLERANCE;
    } else {
      flywheel.setPower(0);
    }

    // Rumble the operator's controller the moment the flywheel becomes ready
    if (ready && !wasReady) gamepad2.rumble(200);
    wasReady = ready;

    // ---- Intake: Cross = in, Circle = out ----
    double ip = 0, sp = 0;
    String intakeState = "stopped";
    if (gamepad2.a)      { ip =  INTAKE_POWER; sp =  SERVO_POWER; intakeState = "IN"; }
    else if (gamepad2.b) { ip = -INTAKE_POWER; sp = -SERVO_POWER; intakeState = "OUT"; }
    intakeMain.setPower(ip);
    leftServo.setPower(sp);
    rightServo.setPower(sp);

    // ---- Transfer: Square = feed (only while READY), Triangle = back ----
    double tp = 0;
    String transferState = "holding";
    if (gamepad2.y) {
      tp = -TRANSFER_POWER;
      transferState = "BACK";
    } else if (gamepad2.x) {
      if (ready) {
        tp = TRANSFER_POWER;
        transferState = "FEEDING";
      } else {
        transferState = flywheelOn ? "waiting for READY" : "flywheel OFF (R2)";
      }
    }
    transfer.setPower(tp);

    // ---- Telemetry ----
    String shooterState = !flywheelOn ? "OFF" : (ready ? "READY" : "spinning up...");
    telemetry.addData("SHOOTER", shooterState);
    telemetry.addData("RPM (actual / target)", "%.0f / %.0f", rpm, targetRpm);
    telemetry.addData("Transfer", transferState);
    telemetry.addData("Intake", intakeState);
    telemetry.addData("Drive", scale < 1.0 ? "SLOW" : "normal");
    telemetry.update();
  }

  @Override
  public void stop() {
    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
    intakeMain.setPower(0);
    leftServo.setPower(0);
    rightServo.setPower(0);
    transfer.setPower(0);
    flywheel.setPower(0);
  }
}