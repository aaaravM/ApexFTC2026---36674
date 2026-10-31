package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret bring-up test.
 *
 * START WITH THE TURRET POINTING STRAIGHT AHEAD: the encoder zeroes at INIT.
 *
 * Manual:  right stick X (gamepad 1) turns the turret slowly. Stick LEFT should turn it LEFT
 *          (counter-clockwise from above) and the angle should go UP.
 * Hold:    D-pad up = 0 deg, D-pad left = +90 deg (left), D-pad right = -90 deg (right).
 *          Moving the stick switches back to manual.
 * Software limits from RobotConstants.Turret are enforced in both modes.
 */
@TeleOp(name = "T8 Turret Test", group = "Test")
public class T8_TurretTest extends OpMode {

  private static final double MANUAL_POWER = 0.3;  // slow, for bring-up
  private static final double STICK_DEADBAND = 0.1;

  private Turret turret;
  private boolean holding = false;

  @Override
  public void init() {
    turret = new Turret(hardwareMap);   // zeroes the encoder: turret must point straight ahead
    turret.setTargetAngleRad(0);
    telemetry.addLine("Turret zeroed. It must be pointing STRAIGHT AHEAD right now.");
    telemetry.update();
  }

  @Override
  public void loop() {
    double angle = turret.getAngleRad();
    double stick = gamepad1.right_stick_x;

    // ---- Choose mode ----
    if (gamepad1.dpad_up)    { turret.setTargetAngleRad(0);                  holding = true; }
    if (gamepad1.dpad_left)  { turret.setTargetAngleRad(Math.toRadians(90));  holding = true; }
    if (gamepad1.dpad_right) { turret.setTargetAngleRad(Math.toRadians(-90)); holding = true; }
    if (Math.abs(stick) > STICK_DEADBAND) holding = false;

    String mode;
    if (holding) {
      // Position control with his PID. 0 = chassis not rotating (robot is still for this test).
      turret.update(0);
      mode = "HOLD " + String.format("%.0f deg", turret.getTargetAngleDeg());
    } else {
      // Stick left (negative X) -> positive power -> should be counter-clockwise / angle UP.
      double power = Math.abs(stick) > STICK_DEADBAND ? -stick * MANUAL_POWER : 0;

      // Enforce the software limits, since setRawPower() bypasses them.
      if (angle >= RobotConstants.Turret.MAX_ANGLE_RAD && power > 0) power = 0;
      if (angle <= RobotConstants.Turret.MIN_ANGLE_RAD && power < 0) power = 0;

      turret.setRawPower(power);
      mode = String.format("MANUAL (power %+.2f)", power);
    }

    boolean atLimit = angle >= RobotConstants.Turret.MAX_ANGLE_RAD
        || angle <= RobotConstants.Turret.MIN_ANGLE_RAD;

    telemetry.addData("Mode", mode);
    telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(angle));
    telemetry.addData("Encoder ticks", turret.getEncoderTicks());
    telemetry.addData("Limits (deg)", "%.0f .. %.0f",
        Math.toDegrees(RobotConstants.Turret.MIN_ANGLE_RAD),
        Math.toDegrees(RobotConstants.Turret.MAX_ANGLE_RAD));
    if (atLimit) telemetry.addLine("*** AT LIMIT ***");
    if (holding) telemetry.addData("Error (deg)", "%+.1f", turret.getErrorDeg());
    telemetry.update();
  }

  @Override
  public void stop() {
    turret.stop();
  }
}