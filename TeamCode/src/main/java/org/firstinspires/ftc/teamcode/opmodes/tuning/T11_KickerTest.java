package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Kicker servo bring-up: find the REST and KICK positions safely.
 *
 * The servo does NOT move until you press Cross (it then goes to the middle, 0.5).
 *   Cross        = enable, go to 0.5
 *   D-pad up/dn  = +/- 0.01 (fine)
 *   Bumpers R/L  = +/- 0.05 (coarse)
 *   Square       = save current position as REST
 *   Triangle     = save current position as KICK
 *   Circle       = toggle between saved REST and KICK (once both are saved)
 *
 * If the servo BUZZES or strains when stopped, it is pushing against a hard stop: back it off.
 */
@TeleOp(name = "T11 Kicker Test", group = "Test")
public class T11_KickerTest extends OpMode {

  private Servo kicker;

  private boolean enabled = false;
  private double position = 0.5;
  private double rest = -1, kick = -1;     // -1 = not saved yet
  private boolean atKick = false;

  private boolean prevA, prevB, prevX, prevY, prevUp, prevDown, prevRB, prevLB;

  @Override
  public void init() {
    kicker = hardwareMap.get(Servo.class, RobotConstants.Names.KICKER);
    // Deliberately NO setPosition() here: the servo stays limp until Cross is pressed.
    telemetry.addLine("Kicker is NOT moving. Press Cross after Play to enable at 0.5.");
    telemetry.update();
  }

  @Override
  public void loop() {
    if (gamepad1.a && !prevA) enabled = true;

    if (enabled) {
      if (gamepad1.dpad_up && !prevUp)        position += 0.01;
      if (gamepad1.dpad_down && !prevDown)    position -= 0.01;
      if (gamepad1.right_bumper && !prevRB)   position += 0.05;
      if (gamepad1.left_bumper && !prevLB)    position -= 0.05;
      position = Math.max(0.0, Math.min(1.0, position));

      if (gamepad1.x && !prevX) rest = position;   // Square
      if (gamepad1.y && !prevY) kick = position;   // Triangle

      if (gamepad1.b && !prevB && rest >= 0 && kick >= 0) {   // Circle
        atKick = !atKick;
        position = atKick ? kick : rest;
      }

      kicker.setPosition(position);
    }

    prevA = gamepad1.a;   prevB = gamepad1.b;   prevX = gamepad1.x;   prevY = gamepad1.y;
    prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
    prevRB = gamepad1.right_bumper; prevLB = gamepad1.left_bumper;

    telemetry.addData("Status", enabled ? "ENABLED" : "not moving (press Cross)");
    telemetry.addData("Position", "%.2f", position);
    telemetry.addData("Saved REST (Square)", rest >= 0 ? String.format("%.2f", rest) : "-");
    telemetry.addData("Saved KICK (Triangle)", kick >= 0 ? String.format("%.2f", kick) : "-");
    if (rest >= 0 && kick >= 0) telemetry.addData("Circle toggles", atKick ? "now at KICK" : "now at REST");
    telemetry.update();
  }
}