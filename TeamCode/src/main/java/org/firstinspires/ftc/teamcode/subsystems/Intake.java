package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Intake: main roller motor only (117 RPM, 50.9:1). The side servos were removed.
 * Positive power = "in". Plain methods for TeleOp, Ivy commands for autos.
 */
public class Intake {

  /** Main roller power, 0..1. */
  public static double MAIN_POWER = 0.8;

  private final DcMotorEx main;

  public Intake(HardwareMap hardwareMap) {
    main = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
    stop();
  }

  public void in()   { main.setPower(MAIN_POWER); }
  public void out()  { main.setPower(-MAIN_POWER); }
  public void stop() { main.setPower(0); }

  /** Main roller speed in encoder ticks per second (motor shaft, before the gearbox). */
  public double getMainTicksPerSec() { return main.getVelocity(); }

  // ---- Ivy commands (autos) ----
  public Command inCommand()   { return instant(this::in); }
  public Command outCommand()  { return instant(this::out); }
  public Command stopCommand() { return instant(this::stop); }
}