package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/*
 * Apex Robotics #36674 - Dual-motor flywheel PIDF tuner
 *
 * ONE control loop, ONE encoder, SAME power to both motors -> motors never fight.
 *   power = (kV * targetRPM + kS) * (12 / batteryVolts) + kP * errorRPM
 *
 * Config names:  "shooter" (has the encoder, Control Hub port 3)
 *                "shooter2" (second flywheel motor - set to the port you use)
 *
 * CONTROLS (gamepad1):
 *   A               = flywheel on/off
 *   Dpad Up/Down    = target RPM +/- 50
 *   Y / X           = kV  up / down   (tune FIRST, with kP = 0)
 *   Right/Left Bumper = kP up / down  (tune SECOND)
 *   B               = zero kP (quick way to test kV alone)
 *   TRANSFER (hold):  Right Trigger = forward, Left Trigger = backward
 *   INTAKE   (hold):  Dpad Right    = forward, Dpad Left    = backward
 *
 * Config names also used: "transfer" (Control Hub port 2), "intake".
 * If "intake" isn't in the config, the tuner still runs - intake buttons just do nothing.
 *
 * TUNING STEPS:
 *   1. Set kP = 0 (press B). Turn on flywheel at your shooting RPM.
 *   2. Adjust kV with Y/X until Actual RPM sits close to Target (within ~50-100).
 *   3. Now raise kP with the bumpers until it recovers quickly after a shot
 *      WITHOUT the RPM bouncing up and down. If it bounces, back kP off.
 *   4. Write the final kV / kP into the constants below.
 */
@TeleOp(name = "Dual Flywheel FF+P Tuner", group = "Test")
public class FlywheelPIDFTuner extends LinearOpMode {

  // 6000 RPM motor (28 ticks/rev) geared 2:1 down -> 56 ticks per FLYWHEEL rev.
  static final double TICKS_PER_FLYWHEEL_REV = 28.0 * 2.0;

  // Starting gains (error in RPM, output in motor power -1..1)
  double kV = 1.0 / 3000.0;  // ~full power at 3000 flywheel RPM
  double kS = 0.0;           // static friction kick, usually 0.0 - 0.05
  double kP = 0.0005;

  static final double KV_STEP = 0.000005;
  static final double KP_STEP = 0.0001;
  static final int RPM_STEP = 50;

  // Low-pass filter on measured RPM (0 = no filter, 0.9 = heavy). Reduces encoder noise.
  static final double FILTER = 0.6;

  // Transfer / intake power when a button is held (0.0 - 1.0)
  static final double TRANSFER_POWER = 0.8;
  static final double INTAKE_POWER = 0.8;

  DcMotorEx shooter, shooter2;
  DcMotor transfer, intake;
  VoltageSensor battery;

  int targetRPM = 2500;
  boolean on = false;
  double filteredRPM = 0;
  boolean pA, pB, pX, pY, pUp, pDown, pRB, pLB;

  @Override
  public void runOpMode() {
    shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
    shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    battery  = hardwareMap.voltageSensor.iterator().next();

    transfer = hardwareMap.get(DcMotor.class, "transfer");
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    transfer.setDirection(DcMotorSimple.Direction.FORWARD); // flip if "forward" goes the wrong way

    intake = hardwareMap.tryGet(DcMotor.class, "intake"); // null if not in config
    if (intake != null) {
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      intake.setDirection(DcMotorSimple.Direction.FORWARD); // flip if needed
    }

    // We do our own control, so run WITHOUT the built-in velocity PID.
    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // encoder still reads
    shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    // Motors facing each other usually need opposite directions.
    // Make sure BOTH push the flywheel the same way AND Actual RPM reads POSITIVE.
    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    telemetry.addLine("Ready. Press START.");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      // ----- buttons (one step per press) -----
      if (gamepad1.a && !pA) on = !on;
      if (gamepad1.dpad_up && !pUp)     targetRPM = Math.min(targetRPM + RPM_STEP, 3000);
      if (gamepad1.dpad_down && !pDown) targetRPM = Math.max(targetRPM - RPM_STEP, 0);
      if (gamepad1.y && !pY) kV += KV_STEP;
      if (gamepad1.x && !pX) kV = Math.max(0, kV - KV_STEP);
      if (gamepad1.right_bumper && !pRB) kP += KP_STEP;
      if (gamepad1.left_bumper && !pLB)  kP = Math.max(0, kP - KP_STEP);
      if (gamepad1.b && !pB) kP = 0;
      pA = gamepad1.a; pB = gamepad1.b; pX = gamepad1.x; pY = gamepad1.y;
      pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
      pRB = gamepad1.right_bumper; pLB = gamepad1.left_bumper;

      // ----- measure (one encoder only) -----
      double rawRPM = shooter.getVelocity() / TICKS_PER_FLYWHEEL_REV * 60.0;
      filteredRPM = FILTER * filteredRPM + (1 - FILTER) * rawRPM;

      // ----- control -----
      double power = 0;
      double error = targetRPM - filteredRPM;
      if (on && targetRPM > 0) {
        double volts = battery.getVoltage();
        double ff = (kV * targetRPM + kS) * (12.0 / volts);
        power = ff + kP * error;
        power = Math.max(0, Math.min(1, power)); // never drive a flywheel backwards
      }

      // SAME power to both motors
      shooter.setPower(power);
      shooter2.setPower(power);

      // ----- transfer (hold triggers) -----
      double transferPower = 0;
      if (gamepad1.right_trigger > 0.3)     transferPower = TRANSFER_POWER;
      else if (gamepad1.left_trigger > 0.3) transferPower = -TRANSFER_POWER;
      transfer.setPower(transferPower);

      // ----- intake (hold dpad left/right) -----
      double intakePower = 0;
      if (gamepad1.dpad_right)     intakePower = INTAKE_POWER;
      else if (gamepad1.dpad_left) intakePower = -INTAKE_POWER;
      if (intake != null) intake.setPower(intakePower);

      // ----- telemetry -----
      telemetry.addData("State", on ? "ON" : "OFF");
      telemetry.addData("Target RPM", targetRPM);
      telemetry.addData("Actual RPM", "%.0f", filteredRPM);
      telemetry.addData("Error", "%.0f", on ? error : 0);
      telemetry.addData("Power", "%.3f", power);
      telemetry.addData("kV", "%.6f", kV);
      telemetry.addData("kP", "%.5f", kP);
      telemetry.addData("Battery", "%.2f V", battery.getVoltage());
      telemetry.addData("Transfer", transferPower > 0 ? "FORWARD" : transferPower < 0 ? "BACKWARD" : "off");
      telemetry.addData("Intake", intake == null ? "not in config"
          : intakePower > 0 ? "FORWARD" : intakePower < 0 ? "BACKWARD" : "off");
      telemetry.update();
    }
    shooter.setPower(0);
    shooter2.setPower(0);
    transfer.setPower(0);
    if (intake != null) intake.setPower(0);
  }
}