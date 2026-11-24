package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Red alliance, left-wall start. 18 in robot: its center must stay >= 9 in from every wall
 * (more when turned diagonally).
 *
 * PEDRO 3 line() HEADING BUG: on line() paths with a heading change, linear() arguments
 * are SWAPPED below. curve() paths use normal order. See TEAM-README.
 */
public class BasicCompAuto {

  private final PoseFactory p = PoseFactory.degrees();

  // ---- Poses ----
  public final Pose start         = p.of(9.0, 108.744, 0);        // flat against left wall
  public final Pose shootLow      = p.of(60.3092, 9.5, 90);       // below red HIVE, facing it
  public final Pose shootLowCtrl  = p.of(26.8678, 47.315, 0);     // control point
  public final Pose cornerApproach= p.of(22.0688, 13.0, 220);     // y 13: diagonal robot needs ~12.7 in
  public final Pose corner        = p.of(9.5, 9.5, 180);          // as far into the corner as fits
  public final Pose crossCtrl     = p.of(15.0, 110.0, 0);         // bows path left, away from the HIVE
  public final Pose shootHigh1    = p.of(55.8242, 129.6298, 270); // above red HIVE, facing it
  public final Pose topPickup     = p.of(47.1268, 134.5, 89);     // max y 135; 89 avoids 180-deg tie
  public final Pose shootHigh2    = p.of(58.7082, 129.0751, 270);
  public final Pose park          = p.of(9.0, 107.1939, 0);       // flat against left wall

  // ---- Paths ----
  /** Start -> shoot below the HIVE. curve(): normal linear(). */
  public Path toShootLow() {
    return curve(start, shootLowCtrl, shootLow).linear(start, shootLow);
  }

  /** -> corner approach. line(): linear SWAPPED. */
  public Path toCornerApproach() {
    return line(shootLow, cornerApproach).linear(cornerApproach, shootLow);
  }

  /** -> into the corner to pick up. line(): linear SWAPPED. */
  public Path toCorner() {
    return line(cornerApproach, corner).linear(corner, cornerApproach);
  }

  /** Corner -> above the HIVE, bowing left around it. curve(): normal linear(). */
  public Path toShootHigh1() {
    return curve(corner, crossCtrl, shootHigh1).linear(corner, shootHigh1);
  }

  /** -> top wall pickup. line(): linear SWAPPED. */
  public Path toTopPickup() {
    return line(shootHigh1, topPickup).linear(topPickup, shootHigh1);
  }

  /** -> back above the HIVE to shoot. line(): linear SWAPPED. */
  public Path toShootHigh2() {
    return line(topPickup, shootHigh2).linear(shootHigh2, topPickup);
  }

  /** -> park. line(): linear SWAPPED. */
  public Path toPark() {
    return line(shootHigh2, park).linear(park, shootHigh2);
  }
}