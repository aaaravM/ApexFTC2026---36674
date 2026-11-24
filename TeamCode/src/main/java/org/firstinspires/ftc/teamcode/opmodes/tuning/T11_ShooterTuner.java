package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Dual-motor flywheel tuner (Flywheel + flywheel_2 on ONE shaft, geared 2:1 down).
 *
 * SHARE/BACK switches stage.
 *
 * STAGE 1 - DIRECTION (20% power, no velocity control)
 *   Hold Cross  = motor 1 (Flywheel) only      Square   = flip motor 1 direction
 *   Hold Circle = motor 2 (flywheel_2) only    Triangle = flip motor 2 direction
 *   Both motors' encoders turn with the shaft, so the screen shows whether they AGREE.
 *
 * STAGE 2 - RUN (both motors, velocity PIDF)
 *   Circle = flywheel on/off          Cross = step test (jump between low and target RPM)
 *   R2 / L2 = target RPM +/- 25
 *   D-pad up/down = select gain (F, P, I, D)    D-pad right/left = gain +/- step
 *   R1 / L1 = step size x10 / /10               Left stick button = reset metrics
 *
 * SAFETY: in Stage 2, power is cut if either motor draws near-stall current, or the two
 * encoders disagree, for more than half a second. Circle clears the fault.
 */
@TeleOp(name = "T11 Shooter Tuner", group = "Test")
public class T11_ShooterTuner extends OpMode {

  private static final double TICKS_PER_REV = RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV; // 56
  private static final double DIRECTION_POWER = 0.2;
  private static final double RPM_STEP = 25;
  private static final double AMP_LIMIT = 8.0;        // stall is ~9.2 A
  private static final double MISMATCH_RPM = 300;     // encoders on one shaft should agree
  private static final double FAULT_SEC = 0.5;

  private enum Stage { DIRECTION, RUN }
  private Stage stage = Stage.DIRECTION;

  private DcMotorEx m1, m2;
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

  // Button edges
  private boolean pBack, pA, pB, pX, pY, pUp, pDown, pLeft, pRight, pRB, pLB, pR2, pL2, pLS;

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
    telemetry.addLine("T13 ready. Starts in STAGE 1 (direction). Share/Back switches stage.");
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

  private void stopBoth() {
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

    // ---- Stage switch (always stops the motors first) ----
    if (back && !pBack) {
      stopBoth();
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
      else        { stopBoth(); running = "stopped"; }

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

      telemetry.addLine("=== STAGE 1: DIRECTION (Share/Back = next stage) ===");
      telemetry.addData("Running", running);
      telemetry.addData("Motor 1 reversed (Square)", rev1);
      telemetry.addData("Motor 2 reversed (Triangle)", rev2);
      telemetry.addData("Motor 1 / 2 RPM", "%.0f / %.0f", rpm1, rpm2);
      telemetry.addData("Encoders", agree);
      telemetry.addLine("Goal: each motor alone spins the flywheel the LAUNCHING way,");
      telemetry.addLine("both RPMs positive, encoders AGREE.");

    } else {
      // ================= STAGE 2: RUN =================
      // On/off and fault clear
      if (b && !pB) {
        if (fault != null) { fault = null; on = false; }
        else { on = !on; if (on) markTargetChange(); }
      }
      // Step test
      if (a && !pA && on) { stepLow = !stepLow; markTargetChange(); }
      // Target RPM
      if (r2 && !pR2) { targetRpm = Math.min(RobotConstants.Shoot.MAX_FLYWHEEL_RPM, targetRpm + RPM_STEP); markTargetChange(); }
      if (l2 && !pL2) { targetRpm = Math.max(0, targetRpm - RPM_STEP); markTargetChange(); }
      // Gain selection and editing
      if (up && !pUp)     selected = (selected + gains.length - 1) % gains.length;
      if (down && !pDown) selected = (selected + 1) % gains.length;
      if (rb && !pRB) gainStep = Math.min(10.0, gainStep * 10);
      if (lb && !pLB) gainStep = Math.max(0.001, gainStep / 10);
      boolean gainChanged = false;
      if (right && !pRight) { gains[selected] += gainStep; gainChanged = true; }
      if (left && !pLeft)   { gains[selected] = Math.max(0, gains[selected] - gainStep); gainChanged = true; }
      if (gainChanged) applyPidf();
      // Reset metrics
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

        // Metrics
        if (!reached && ready) { reached = true; reachSec = (System.nanoTime() - changeNano) / 1e9; }
        if (reached) {
          overshoot = Math.max(overshoot, rpm - tgt);
          lowestAfterReach = Math.min(lowestAfterReach, rpm);
        }
      } else {
        stopBoth();
      }

      String state = fault != null ? "FAULT: " + fault + " (Circle clears)"
          : !on ? "OFF (Circle)" : (ready ? "READY" : "spinning...");

      telemetry.addLine("=== STAGE 2: RUN (Share/Back = direction stage) ===");
      telemetry.addData("Flywheel", state);
      telemetry.addData("Target (R2/L2)", "%.0f%s", tgt, stepLow ? "  [STEP LOW]" : "");
      telemetry.addData("Actual (avg)", "%.0f   error %+.0f", rpm, rpm - tgt);
      telemetry.addData("Motor 1 / 2 RPM", "%.0f / %.0f", rpm1, rpm2);
      telemetry.addData("Motor 1 / 2 amps", "%.1f / %.1f", amps1, amps2);
      telemetry.addLine();
      for (int i = 0; i < gains.length; i++) {
        telemetry.addData((i == selected ? "> " : "  ") + gainNames[i], "%.4f", gains[i]);
      }
      telemetry.addData("Step (R1/L1)", "%.3f", gainStep);
      telemetry.addLine();
      telemetry.addData("Time to READY", reachSec < 0 ? "-" : String.format("%.2f s", reachSec));
      telemetry.addData("Overshoot", reached ? String.format("%+.0f RPM", overshoot) : "-");
      telemetry.addData("Lowest after READY", lowestAfterReach == Double.MAX_VALUE ? "-" : String.format("%.0f", lowestAfterReach));
    }

    telemetry.update();

    pBack = back; pA = a; pB = b; pX = x; pY = y; pUp = up; pDown = down;
    pLeft = left; pRight = right; pRB = rb; pLB = lb; pR2 = r2; pL2 = l2; pLS = ls;
  }

  @Override
  public void stop() {
    stopBoth();
  }
}