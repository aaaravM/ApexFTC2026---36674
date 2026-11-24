package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Dual-motor flywheel tuner (Flywheel + flywheel_2 on ONE shaft, geared 2:1 down),
 * with intake and transfer controls for live shot testing.
 *
 * GAMEPAD 1 - SHOOTER.  SHARE/BACK switches stage.
 *   STAGE 1 - DIRECTION (20% power, no velocity control)
 *     Hold Cross  = motor 1 (Flywheel) only      Square   = flip motor 1 direction
 *     Hold Circle = motor 2 (flywheel_2) only    Triangle = flip motor 2 direction
 *   STAGE 2 - RUN (both motors, velocity PIDF)
 *     Circle = on/off   Cross = step test   R2/L2 = target RPM +/- 25
 *     D-pad up/down = select gain   D-pad right/left = gain +/-   R1/L1 = step x10 / /10
 *     Left stick button = reset metrics
 *
 * GAMEPAD 2 - INTAKE + TRANSFER (both stages, hold to run)
 *   Cross / Circle = intake in / reverse      Square / Triangle = transfer up / reverse
 *   D-pad up/down = intake power +/- 10%      D-pad right/left = transfer power +/- 10%
 *
 * SAFETY: in Stage 2, the flywheel cuts out if either motor draws near-stall current, or the two
 * encoders disagree, for more than half a second. Circle clears the fault.
 */
@TeleOp(name = "T13 Shooter Tuner", group = "Test")
public class T13_ShooterTuner extends OpMode {

  private static final double TICKS_PER_REV = RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV; // 56
  private static final double DIRECTION_POWER = 0.2;
  private static final double RPM_STEP = 25;
  private static final double AMP_LIMIT = 8.0;        // stall is ~9.2 A
  private static final double MISMATCH_RPM = 300;     // encoders on one shaft should agree
  private static final double FAULT_SEC = 0.5;

  private enum Stage { DIRECTION, RUN }
  private Stage stage = Stage.DIRECTION;

  private DcMotorEx m1, m2, intakeMain, transfer;
  private boolean rev1 = RobotConstants.Shoot.REVERSE_FLYWHEEL;
  private boolean rev2 = RobotConstants.Shoot.REVERSE_FLYWHEEL_2;

  // Gains, in the order they're normally tuned
  private final String[] gainNames = {"F", "P", "I", "D"};
  private final double[] gains = {
      RobotConstants.Shoot.FLYWHEEL_F, RobotConstants.Shoot.FLYWHEEL_P,
      RobotConstants.Shoot.FLYWHEEL_I, RobotConstants.Shoot.FLYWHEEL_D};
  private int selected = 0;
  private double gainStep = 0.1;

  // Run state
  private boolean on = false;
  private boolean stepLow = false;
  private double targetRpm = 2500;
  private String fault = null;
  private long faultStartNano = 0;

  // Metrics
  private long changeNano = 0;
  private boolean reached = false;
  private double reachSec = -1, overshoot = 0, lowestAfterReach = Double.MAX_VALUE;

  // Intake / transfer
  private double intakePower = 0.8;
  private double transferPower = 1.0;

  // Button edges, gamepad 1
  private boolean pBack, pA, pB, pX, pY, pUp, pDown, pLeft, pRight, pRB, pLB, pR2, pL2, pLS;
  // Button edges, gamepad 2
  private boolean p2Up, p2Down, p2Left, p2Right;

  @Override
  public void init() {
    m1 = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
    m2 = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL_2);
    for (DcMotorEx m : new DcMotorEx[]{m1, m2}) {
      m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    applyDirections();

    intakeMain = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    intakeMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intakeMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FEEDER);
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    telemetry.addLine("T13 ready. Starts in STAGE 1 (direction). GP1 Share/Back switches stage.");
    telemetry.update();
  }

  private void applyDirections() {
    m1.setDirection(rev1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    m2.setDirection(rev2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  private void applyPidf() {
    // gains[] order is F, P, I, D; the SDK wants P, I, D, F
    m1.setVelocityPIDFCoefficients(gains[1], gains[2], gains[3], gains[0]);
    m2.setVelocityPIDFCoefficients(gains[1], gains[2], gains[3], gains[0]);
  }

  private void stopFlywheel() {
    m1.setPower(0);
    m2.setPower(0);
  }

  private double target() {
    return stepLow ? targetRpm * 0.6 : targetRpm;
  }

  private void markTargetChange() {
    changeNano = System.nanoTime();
    reached = false;
    reachSec = -1;
    overshoot = 0;
    lowestAfterReach = Double.MAX_VALUE;
  }

  @Override
  public void loop() {
    boolean back = gamepad1.back, a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
    boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
    boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
    boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
    boolean r2 = gamepad1.right_trigger > 0.5, l2 = gamepad1.left_trigger > 0.5;
    boolean ls = gamepad1.left_stick_button;

    // ---- Stage switch (always stops the flywheel first) ----
    if (back && !pBack) {
      stopFlywheel();
      on = false;
      fault = null;
      if (stage == Stage.DIRECTION) {
        stage = Stage.RUN;
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidf();
      } else {
        stage = Stage.DIRECTION;
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }
    }

    double v1 = m1.getVelocity(), v2 = m2.getVelocity();           // motor ticks/sec
    double rpm1 = v1 / TICKS_PER_REV * 60.0, rpm2 = v2 / TICKS_PER_REV * 60.0;
    double rpm = (rpm1 + rpm2) / 2.0;
    double amps1 = m1.getCurrent(CurrentUnit.AMPS), amps2 = m2.getCurrent(CurrentUnit.AMPS);

    if (stage == Stage.DIRECTION) {
      // ================= STAGE 1: DIRECTION =================
      String running;
      if (a)      { m1.setPower(DIRECTION_POWER); m2.setPower(0); running = "MOTOR 1 only (Flywheel)"; }
      else if (b) { m1.setPower(0); m2.setPower(DIRECTION_POWER); running = "MOTOR 2 only (flywheel_2)"; }
      else        { stopFlywheel(); running = "stopped"; }

      // Flip directions only while stopped
      if (!a && !b) {
        if (x && !pX) { rev1 = !rev1; applyDirections(); }
        if (y && !pY) { rev2 = !rev2; applyDirections(); }
      }

      String agree = "-";
      if (Math.abs(v1) > 50 && Math.abs(v2) > 50) {
        agree = (Math.signum(v1) == Math.signum(v2))
            ? "AGREE (directions match)" : "DISAGREE -> flip motor 2 (Triangle)";
      }

      telemetry.addLine("=== STAGE 1: DIRECTION (GP1 Share/Back = next stage) ===");
      telemetry.addData("Running", running);
      telemetry.addData("Motor 1 reversed (Square)", rev1);
      telemetry.addData("Motor 2 reversed (Triangle)", rev2);
      telemetry.addData("Motor 1 / 2 RPM", "%.0f / %.0f", rpm1, rpm2);
      telemetry.addData("Encoders", agree);
      telemetry.addLine("Goal: each motor alone spins the flywheel the LAUNCHING way,");
      telemetry.addLine("both RPMs positive, encoders AGREE.");

    } else {
      // ================= STAGE 2: RUN =================
      if (b && !pB) {
        if (fault != null) { fault = null; on = false; }
        else { on = !on; if (on) markTargetChange(); }
      }
      if (a && !pA && on) { stepLow = !stepLow; markTargetChange(); }
      if (r2 && !pR2) { targetRpm = Math.min(RobotConstants.Shoot.MAX_FLYWHEEL_RPM, targetRpm + RPM_STEP); markTargetChange(); }
      if (l2 && !pL2) { targetRpm = Math.max(0, targetRpm - RPM_STEP); markTargetChange(); }
      if (up && !pUp)     selected = (selected + gains.length - 1) % gains.length;
      if (down && !pDown) selected = (selected + 1) % gains.length;
      if (rb && !pRB) gainStep = Math.min(10.0, gainStep * 10);
      if (lb && !pLB) gainStep = Math.max(0.001, gainStep / 10);
      boolean gainChanged = false;
      if (right && !pRight) { gains[selected] += gainStep; gainChanged = true; }
      if (left && !pLeft)   { gains[selected] = Math.max(0, gains[selected] - gainStep); gainChanged = true; }
      if (gainChanged) applyPidf();
      if (ls && !pLS) markTargetChange();

      // Safety: near-stall current or encoders disagreeing = cut power
      if (on && fault == null) {
        String problem = null;
        if (amps1 > AMP_LIMIT || amps2 > AMP_LIMIT) problem = "CURRENT near stall";
        else if (Math.abs(rpm1 - rpm2) > MISMATCH_RPM) problem = "ENCODERS DISAGREE (fighting or slipping?)";
        if (problem != null) {
          if (faultStartNano == 0) faultStartNano = System.nanoTime();
          if ((System.nanoTime() - faultStartNano) / 1e9 > FAULT_SEC) fault = problem;
        } else {
          faultStartNano = 0;
        }
      }

      double tgt = target();
      boolean ready = false;
      if (on && fault == null) {
        double tps = tgt / 60.0 * TICKS_PER_REV;   // motor ticks/sec
        m1.setVelocity(tps);
        m2.setVelocity(tps);
        ready = Math.abs(rpm - tgt) <= RobotConstants.Shoot.RPM_TOLERANCE;

        if (!reached && ready) { reached = true; reachSec = (System.nanoTime() - changeNano) / 1e9; }
        if (reached) {
          overshoot = Math.max(overshoot, rpm - tgt);
          lowestAfterReach = Math.min(lowestAfterReach, rpm);
        }
      } else {
        stopFlywheel();
      }

      String state = fault != null ? "FAULT: " + fault + " (Circle clears)"
          : !on ? "OFF (Circle)" : (ready ? "READY" : "spinning...");

      telemetry.addLine("=== STAGE 2: RUN (GP1 Share/Back = direction stage) ===");
      telemetry.addData("Flywheel", state);
      telemetry.addData("Target (R2/L2)", "%.0f%s", tgt, stepLow ? "  [STEP LOW]" : "");
      telemetry.addData("Actual (avg)", "%.0f   error %+.0f", rpm, rpm - tgt);
      telemetry.addData("Motor 1 / 2 RPM", "%.0f / %.0f", rpm1, rpm2);
      telemetry.addData("Motor 1 / 2 amps", "%.1f / %.1f", amps1, amps2);
      telemetry.addLine();
      telemetry.addLine("--- PIDF (D-pad up/down select, right/left change) ---");
      for (int i = 0; i < gains.length; i++) {
        telemetry.addData((i == selected ? "> " : "  ") + gainNames[i], "%.4f", gains[i]);
      }
      telemetry.addData("Step (R1/L1)", "%.3f", gainStep);
      telemetry.addLine();
      telemetry.addData("Time to READY", reachSec < 0 ? "-" : String.format("%.2f s", reachSec));
      telemetry.addData("Overshoot", reached ? String.format("%+.0f RPM", overshoot) : "-");
      telemetry.addData("Lowest after READY", lowestAfterReach == Double.MAX_VALUE ? "-" : String.format("%.0f", lowestAfterReach));
    }

    // ================= GAMEPAD 2: INTAKE + TRANSFER (both stages) =================
    boolean u2 = gamepad2.dpad_up, d2 = gamepad2.dpad_down, l2d = gamepad2.dpad_left, r2d = gamepad2.dpad_right;
    if (u2 && !p2Up)     intakePower   = Math.min(1.0, intakePower + 0.1);
    if (d2 && !p2Down)   intakePower   = Math.max(0.1, intakePower - 0.1);
    if (r2d && !p2Right) transferPower = Math.min(1.0, transferPower + 0.1);
    if (l2d && !p2Left)  transferPower = Math.max(0.1, transferPower - 0.1);
    p2Up = u2; p2Down = d2; p2Left = l2d; p2Right = r2d;

    String intakeState = "stopped";
    double ip = 0;
    if (gamepad2.a)      { ip =  intakePower; intakeState = "IN"; }
    else if (gamepad2.b) { ip = -intakePower; intakeState = "REVERSE"; }
    intakeMain.setPower(ip);

    String transferState = "holding";
    double tp = 0;
    if (gamepad2.x)      { tp =  transferPower; transferState = "UP"; }
    else if (gamepad2.y) { tp = -transferPower; transferState = "REVERSE"; }
    transfer.setPower(tp);

    telemetry.addLine();
    telemetry.addLine("--- GP2: intake + transfer ---");
    telemetry.addData("Intake (Cross/Circle)", "%s @ %.0f%%", intakeState, intakePower * 100);
    telemetry.addData("Transfer (Square/Triangle)", "%s @ %.0f%%", transferState, transferPower * 100);

    telemetry.update();

    pBack = back; pA = a; pB = b; pX = x; pY = y; pUp = up; pDown = down;
    pLeft = left; pRight = right; pRB = rb; pLB = lb; pR2 = r2; pL2 = l2; pLS = ls;
  }

  @Override
  public void stop() {
    stopFlywheel();
    intakeMain.setPower(0);
    transfer.setPower(0);
  }
}