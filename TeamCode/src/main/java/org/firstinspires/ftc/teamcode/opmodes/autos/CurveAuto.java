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

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Test auto: curve out (tangent heading) and curve back (linear heading), ending at the start.
 * Needs ~4 ft clear ahead AND to the left. Control points bend the curve; the robot never visits them.
 */
@Autonomous(name = "Curve Auto", group = "Test")
public class CurveAuto extends OpMode {

  private Follower follower;
  private final PoseFactory p = PoseFactory.degrees();

  private final Pose start  = p.of(24, 24, 0);
  private final Pose pointB = p.of(48, 48, 90);

  private final Pose controlOut  = p.of(48, 24, 0);
  private final Pose controlBack = p.of(24, 48, 0);

  // Out: sweep forward then left, facing the direction of travel
  private Path curveOut() { return curve(start, controlOut, pointB).tangent(); }

  // Back: return while turning 90 -> 0. curve() needs NO linear() swap (only line() is affected).
  private Path curveBack() { return curve(pointB, controlBack, start).linear(pointB, start); }

  private Command autoRoutine() {
    return sequential(
        follow(follower, curveOut()),
        follow(follower, curveBack())
    );
  }

  @Override
  public void init() {
    Scheduler.reset();
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
}