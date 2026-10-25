package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Test auto: drive 24" straight forward. Needs ~3 ft clear ahead. */
@Autonomous(name = "First Auto", group = "Test")
public class FirstAuto extends OpMode {

  private Follower follower;
  private final PoseFactory p = PoseFactory.degrees();

  private final Pose startPose = p.of(24, 24, 0);
  private final Pose endPose   = p.of(48, 24, 0);

  private Path driveForward() {
    return line(startPose, endPose).linear(startPose, endPose);
  }

  @Override
  public void init() {
    Scheduler.reset();
    follower = PedroConstants.createFollower(hardwareMap);
    follower.setPose(startPose);
    follower.update();
  }

  @Override
  public void start() {
    schedule(follow(follower, driveForward()));
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