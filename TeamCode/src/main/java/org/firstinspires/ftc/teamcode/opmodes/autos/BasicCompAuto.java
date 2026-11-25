package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Red alliance paths (from the Pedro visualizer). 18 in robot: center must stay >= 9 in from walls.
 *
 * PEDRO 3 line() HEADING BUG: on line() paths that change heading, the linear() arguments are
 * SWAPPED below. curve() paths use normal order. See TEAM-README.
 * 269 / 89 instead of 270 / 90 on the turn-around points: an exact 180-deg turn has no
 * preferred direction, so these make the robot turn one way and unwind the same way.
 */
public class BasicCompAuto {

  private final PoseFactory p = PoseFactory.degrees();

  public final Pose start          = p.of(9.4147, 107.2321, 0);
  public final Pose shootLow       = p.of(59.8671, 13.1929, 90);
  public final Pose shootLowCtrl   = p.of(23.6312, 43.4605, 0);   // control point
  public final Pose corner         = p.of(9.0, 10.9023, 269);     // x 9.0 (was 8.66: in the wall)
  public final Pose shootLow2      = p.of(59.5031, 13.0288, 90);
  public final Pose leftSide       = p.of(9.6872, 46.57, 180);
  public final Pose shootHigh      = p.of(57.9766, 129.3259, 270);
  public final Pose shootHighCtrl  = p.of(15.0, 105.0, 0);        // control point (was 22.7, 88.2: HIVE)
  public final Pose topApproach    = p.of(47.3436, 127.0178, 89);
  public final Pose topPickup      = p.of(47.2598, 133.1132, 89);
  public final Pose shootHigh2     = p.of(57.9466, 129.1822, 270);
  public final Pose shootHigh2Ctrl = p.of(46.7051, 117.342, 0);   // control point
  public final Pose park           = p.of(9.4118, 107.414, 0);

  /** 1: start -> low shooting spot. curve(): normal. */
  public Path path1() { return curve(start, shootLowCtrl, shootLow).linear(start, shootLow); }

  /** 2: -> corner pickup. line(), heading changes: SWAPPED. */
  public Path path2() { return line(shootLow, corner).linear(corner, shootLow); }

  /** 3: corner -> low shooting spot. line(): SWAPPED. */
  public Path path3() { return line(corner, shootLow2).linear(shootLow2, corner); }

  /** 4: -> left side. line(): SWAPPED. */
  public Path path4() { return line(shootLow2, leftSide).linear(leftSide, shootLow2); }

  /** 5: left side -> high shooting spot, curving around the HIVE. curve(): normal. */
  public Path path5() { return curve(leftSide, shootHighCtrl, shootHigh).linear(leftSide, shootHigh); }

  /** 6: -> top pickup approach. line(): SWAPPED. */
  public Path path6() { return line(shootHigh, topApproach).linear(topApproach, shootHigh); }

  /** 7: -> into top pickup. line(), same heading: no swap needed. */
  public Path path7() { return line(topApproach, topPickup).linear(topApproach, topPickup); }

  /** 8: top pickup -> high shooting spot. curve(): normal. */
  public Path path8() { return curve(topPickup, shootHigh2Ctrl, shootHigh2).linear(topPickup, shootHigh2); }

  /** 9: -> park. line(): SWAPPED. */
  public Path path9() { return line(shootHigh2, park).linear(park, shootHigh2); }
}