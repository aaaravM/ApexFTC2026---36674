package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Intake: main roller motor (117 RPM, 50.9:1) plus two continuous side servos.
 * Positive power = "in" for all three parts (the right servo is mirrored, so it is reversed here).
 *
 * Plain methods for TeleOp, Ivy commands for autos.
 */
public class Intake {

  /** Main roller power, 0..1. */
  public static double MAIN_POWER = 0.8;
  /** Side servo power, 0..1. */
  public static double SERVO_POWER = 1.0;

  private final DcMotorEx main;
  private final CRServo left;
  private final CRServo right;

  public Intake(HardwareMap hardwareMap) {
    main  = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    left  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);
    right = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);

    // Mirrored servo: spins "out" on positive power unless reversed.
    right.setDirection(DcMotorSimple.Direction.REVERSE);

    stop();
  }

  // ---- Plain actions (TeleOp) ----

  public void in() {
    main.setPower(MAIN_POWER);
    left.setPower(SERVO_POWER);
    right.setPower(SERVO_POWER);
  }

  public void out() {
    main.setPower(-MAIN_POWER);
    left.setPower(-SERVO_POWER);
    right.setPower(-SERVO_POWER);
  }

  public void stop() {
    main.setPower(0);
    left.setPower(0);
    right.setPower(0);
  }

  /** Main roller speed in encoder ticks per second (motor shaft, before the gearbox). */
  public double getMainTicksPerSec() {
    return main.getVelocity();
  }

  // ---- Ivy commands (autos) ----

  public Command inCommand()   { return instant(this::in); }
  public Command outCommand()  { return instant(this::out); }
  public Command stopCommand() { return instant(this::stop); }
}