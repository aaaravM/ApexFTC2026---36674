package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Curve Auto with the intake running on the way out.
 * Place a pollen ~18" ahead and ~6" left of the robot's starting CENTER (the curve's midpoint).
 * Assumes the intake is on the FRONT of the robot.
 */
@Autonomous(name = "Intake Curve Auto", group = "Test")
public class IntakeCurveAuto extends OpMode {

  private Follower follower;
  private Intake intake;
  private final PoseFactory p = PoseFactory.degrees();

  private final Pose start  = p.of(24, 24, 0);
  private final Pose pointB = p.of(48, 48, 90);

  private final Pose controlOut  = p.of(48, 24, 0);
  private final Pose controlBack = p.of(24, 48, 0);

  private Path curveOut()  { return curve(start, controlOut, pointB).tangent(); }
  private Path curveBack() { return curve(pointB, controlBack, start).linear(pointB, start); }

  private Command autoRoutine() {
    return sequential(
        intake.inCommand(),               // 1. start intake
        follow(follower, curveOut()),     // 2. curve out through the pollen
        waitMs(500),                      // 3. let it finish going in
        intake.stopCommand(),             // 4. stop intake
        follow(follower, curveBack())     // 5. carry it back to start
    );
  }

  @Override
  public void init() {
    Scheduler.reset();
    intake = new Intake(hardwareMap);
    follower = PedroConstants.createFollower(hardwareMap);
    follower.setPose(start);
    follower.update();
  }

  @Override
  public void start() {
    schedule(autoRoutine());
  }

  @Override
  public void loop() {
    follower.update();
    Scheduler.execute();

    telemetry.addData("X", follower.pose().x());
    telemetry.addData("Y", follower.pose().y());
    telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
    telemetry.update();
  }

  @Override
  public void stop() {
    intake.stop();
  }
}