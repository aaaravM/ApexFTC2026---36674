package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Red alliance, left-wall parking start (from the Pedro visualizer).
 *   path1: parking -> shooting spot below the red HIVE
 *   path2: shooting spot -> bottom-left corner to pick up balls
 *   path3: corner -> back to parking (driving in reverse)
 */
public class BasicCompAuto {

  private final PoseFactory p = PoseFactory.degrees();

  /** Start in the red parking zone, facing down the field. */
  public final Pose start = p.of(9.8972, 109.4261, 270);

  /** Shooting spot below the red HIVE. 89 (not 90) so the 180-deg turn always goes the same way. */
  public final Pose shoot = p.of(55.1814, 29.9668, 89);
  public final Pose shootControl = p.of(26.5977, 42.45, 0);      // control point: heading ignored

  /** Same spot as shoot, turned to face the corner before driving there. */
  public final Pose leaveShoot = p.of(55.1814, 29.9668, 180);

  /** Bottom-left corner, facing the balls along the bottom wall. */
  public final Pose corner = p.of(11.0446, 11.0373, 270);

  /** Back in parking. x = 9.9 (was 8.09) keeps an 18" robot off the wall. */
  public final Pose park = p.of(9.8972, 107.4536, -73.2605);
  public final Pose parkControl = p.of(23.2642, 56.9901, 0);     // control point: heading ignored

  /** Parking -> shooting spot, turning 270 -> 89 on the way. curve(): no linear() swap. */
  public Path toShoot() {
    return curve(start, shootControl, shoot).linear(start, shoot);
  }

  /** Shooting spot -> corner, turning 180 -> 270.
   *  linear() args SWAPPED: Pedro 3 line() heading bug (see TEAM-README). */
  public Path toCorner() {
    return line(leaveShoot, corner).linear(corner, leaveShoot);
  }

  /** Corner -> parking, driving in reverse. Ends at about -73 deg (matches park). */
  public Path toPark() {
    return curve(corner, parkControl, park).reverseTangent();
  }
}