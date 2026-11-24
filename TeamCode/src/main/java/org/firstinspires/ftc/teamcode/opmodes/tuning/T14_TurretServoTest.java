package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Turret bring-up: two continuous servos (turret1, turret2) driving one turret gear, no encoder.
 *
 * KEEP THE TURRET AWAY FROM ITS LIMITS. There are no software limits in this test.
 *
 *   D-pad left/right (hold) = both servos; LEFT should turn the turret LEFT (counter-clockwise)
 *   Cross (hold)  = servo 1 only (servo 2 limp)     Square   = flip servo 1 (while stopped)
 *   Circle (hold) = servo 2 only (servo 1 limp)     Triangle = flip servo 2 (while stopped)
 *   R1 / L1 = power +/- 0.1
 *
 * If the turret BUZZES or barely moves with both servos running, they are fighting: flip one.
 */
@TeleOp(name = "T14 Turret Servo Test", group = "Test")
public class T14_TurretServoTest extends OpMode {

  private CRServoImplEx s1, s2;
  private boolean rev1 = false, rev2 = false;
  private double power = 0.3;

  private long holdStartNano = 0;
  private double lastHoldSec = 0;
  private String lastHoldDir = "-";

  private boolean pX, pY, pRB, pLB, pHolding;

  @Override
  public void init() {
    s1 = hardwareMap.get(CRServoImplEx.class, RobotConstants.Names.TURRET_1);
    s2 = hardwareMap.get(CRServoImplEx.class, RobotConstants.Names.TURRET_2);
    applyDirections();
    telemetry.addLine("T14 ready. Turret should start pointing straight ahead, away from its limits.");
    telemetry.update();
  }

  private void applyDirections() {
    s1.setDirection(rev1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    s2.setDirection(rev2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  private void drive(double p1, boolean on1, double p2, boolean on2) {
    if (on1) { s1.setPwmEnable(); s1.setPower(p1); } else { s1.setPwmDisable(); }
    if (on2) { s2.setPwmEnable(); s2.setPower(p2); } else { s2.setPwmDisable(); }
  }

  @Override
  public void loop() {
    boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
    boolean a = gamepad1.a, b = gamepad1.b;
    boolean idle = !left && !right && !a && !b;

    // Power level
    if (gamepad1.right_bumper && !pRB) power = Math.min(1.0, power + 0.1);
    if (gamepad1.left_bumper && !pLB)  power = Math.max(0.1, power - 0.1);
    pRB = gamepad1.right_bumper; pLB = gamepad1.left_bumper;

    // Direction flips, only while stopped
    if (idle) {
      if (gamepad1.x && !pX) { rev1 = !rev1; applyDirections(); }
      if (gamepad1.y && !pY) { rev2 = !rev2; applyDirections(); }
    }
    pX = gamepad1.x; pY = gamepad1.y;

    String mode;
    if (left)       { drive( power, true,  power, true);  mode = "BOTH +  (should turn LEFT)"; }
    else if (right) { drive(-power, true, -power, true);  mode = "BOTH -  (should turn RIGHT)"; }
    else if (a)     { drive( power, true,  0,     false); mode = "SERVO 1 only (servo 2 limp)"; }
    else if (b)     { drive( 0,     false, power, true);  mode = "SERVO 2 only (servo 1 limp)"; }
    else            { drive( 0,     true,  0,     true);  mode = "stopped"; }

    // Timing for both-servo holds (to measure turret speed)
    boolean holding = left || right;
    if (holding && !pHolding) holdStartNano = System.nanoTime();
    if (!holding && pHolding) {
      lastHoldSec = (System.nanoTime() - holdStartNano) / 1e9;
      lastHoldDir = gamepad1.dpad_left ? "LEFT" : "the last direction";
    }
    pHolding = holding;
    double currentHold = holding ? (System.nanoTime() - holdStartNano) / 1e9 : 0;

    telemetry.addData("Mode", mode);
    telemetry.addData("Power (R1/L1)", "%.1f", power);
    telemetry.addData("Servo 1 reversed (Square)", rev1);
    telemetry.addData("Servo 2 reversed (Triangle)", rev2);
    telemetry.addLine();
    telemetry.addData("Holding for", holding ? String.format("%.2f s", currentHold) : "-");
    telemetry.addData("Last hold lasted", "%.2f s", lastHoldSec);
    telemetry.addLine();
    telemetry.addLine("Goal: each servo ALONE turns the turret the same way;");
    telemetry.addLine("both together turn it LEFT on D-pad left, smoothly, no buzzing.");
    telemetry.update();
  }

  @Override
  public void stop() {
    s1.setPower(0);
    s2.setPower(0);
  }
}