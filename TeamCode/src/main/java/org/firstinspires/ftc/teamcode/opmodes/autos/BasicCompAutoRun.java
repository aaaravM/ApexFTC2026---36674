package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Runs the BasicCompAuto paths in order. DRIVE-ONLY for now: shooting and intake are short
 * pauses (marked TODO) so the paths can be tested before mechanisms are wired in.
 *
 * Start: red parking zone, robot flat against the left wall, facing 0 deg.
 */
@Autonomous(name = "Basic Comp Auto (Red) - paths only", group = "Match")
public class BasicCompAutoRun extends OpMode {

  private Follower follower;
  private final BasicCompAuto paths = new BasicCompAuto();
  private String step = "init";

  private Command label(String name) {
    return instant(() -> step = name);
  }

  private Command routine() {
    return sequential(
        label("1: to low shooting spot"),    follow(follower, paths.path1()),
        label("   SHOOT (TODO)"),             waitMs(1000),
        label("2: to corner"),                follow(follower, paths.path2()),
        label("   INTAKE (TODO)"),            waitMs(1000),
        label("3: back to low spot"),         follow(follower, paths.path3()),
        label("   SHOOT (TODO)"),             waitMs(1000),
        label("4: to left side"),             follow(follower, paths.path4()),
        label("   INTAKE (TODO)"),            waitMs(1000),
        label("5: around HIVE to high spot"), follow(follower, paths.path5()),
        label("   SHOOT (TODO)"),             waitMs(1000),
        label("6: to top approach"),          follow(follower, paths.path6()),
        label("7: into top pickup"),          follow(follower, paths.path7()),
        label("   INTAKE (TODO)"),            waitMs(1000),
        label("8: back to high spot"),        follow(follower, paths.path8()),
        label("   SHOOT (TODO)"),             waitMs(1000),
        label("9: park"),                     follow(follower, paths.path9()),
        label("DONE")
    );
  }

  @Override
  public void init() {
    Scheduler.reset();
    follower = PedroConstants.createFollower(hardwareMap);
    follower.setPose(paths.start);
    follower.update();
    telemetry.addLine("Robot must be in red parking, flat against the left wall, facing 0 deg.");
    telemetry.update();
  }

  @Override
  public void start() {
    schedule(routine());
  }

  @Override
  public void loop() {
    follower.update();
    Scheduler.execute();

    telemetry.addData("Step", step);
    telemetry.addData("X", "%.1f", follower.pose().x());
    telemetry.addData("Y", "%.1f", follower.pose().y());
    telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
    telemetry.update();
  }
}