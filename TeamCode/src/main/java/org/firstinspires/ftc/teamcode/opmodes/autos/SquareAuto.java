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
 * Test auto: 24" square that ends where it started. Needs ~4 ft clear ahead AND to the left.
 *
 * PEDRO 3 KNOWN ISSUE: on line() paths, linear(a, b) interpolates heading BACKWARDS.
 * Workaround: write linear(b, a) on line() paths whose headings differ. curve() is NOT affected.
 * If a Pedro update fixes this, legs 3 and 4 below must be swapped back.
 */
@Autonomous(name = "Square Auto", group = "Test")
public class SquareAuto extends OpMode {

  private Follower follower;
  private final PoseFactory p = PoseFactory.degrees();

  private final Pose start  = p.of(24, 24, 0);
  private final Pose pointA = p.of(48, 24, 0);
  private final Pose pointB = p.of(48, 48, 0);
  private final Pose pointC = p.of(24, 48, 90);

  // Leg 1: forward 24"
  private Path leg1() { return line(start, pointA).linear(start, pointA); }

  // Leg 2: strafe left 24", still facing forward
  private Path leg2() { return line(pointA, pointB).linear(pointA, pointB); }

  // Leg 3: back up while turning left 90 (linear args swapped: line() heading bug)
  private Path leg3() { return line(pointB, pointC).linear(pointC, pointB); }

  // Leg 4: return to start while turning back to 0 (linear args swapped: line() heading bug)
  private Path leg4() { return line(pointC, start).linear(start, pointC); }

  private Command autoRoutine() {
    return sequential(
        follow(follower, leg1()),
        follow(follower, leg2()),
        follow(follower, leg3()),
        follow(follower, leg4())
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