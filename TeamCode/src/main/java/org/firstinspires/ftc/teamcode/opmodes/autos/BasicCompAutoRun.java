package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Runs the BasicCompAuto paths in order. DRIVE-ONLY for now: the shooting and intake steps
 * are short pauses (marked TODO) so the paths can be tested before the mechanisms are wired in.
 *
 * Start: red parking zone, robot flat against the left wall, facing 0 deg (toward the field).
 */
@Autonomous(name = "Basic Comp Auto (Red) - paths only", group = "Match")
public class BasicCompAutoRun extends OpMode {

  private Follower follower;
  private final BasicCompAuto paths = new BasicCompAuto();
  private String step = "init";

  /** Sets the telemetry label, so you can see which leg is running. */
  private Command label(String name) {
    return com.pedropathing.ivy.commands.Commands.instant(() -> step = name);
  }

  private Command routine() {
    return sequential(
        label("1: to low shooting spot"),   follow(follower, paths.toShootLow()),
        label("   SHOOT (TODO)"),            waitMs(1000),

        label("2: to corner approach"),      follow(follower, paths.toCornerApproach()),
        label("3: into corner"),             follow(follower, paths.toCorner()),
        label("   INTAKE (TODO)"),           waitMs(1000),

        label("4: around HIVE to high spot"),follow(follower, paths.toShootHigh1()),
        label("   SHOOT (TODO)"),            waitMs(1000),

        label("5: to top pickup"),           follow(follower, paths.toTopPickup()),
        label("   INTAKE (TODO)"),           waitMs(1000),

        label("6: back to high spot"),       follow(follower, paths.toShootHigh2()),
        label("   SHOOT (TODO)"),            waitMs(1000),

        label("7: park"),                    follow(follower, paths.toPark()),
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