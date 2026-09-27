package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The two BIOBUZZ scoring elements, with the physical properties that change how we shoot them.
 *
 * <p>Official specs (AndyMark am-5851 / am-5852, full field set = 40 POLLEN, 8 red + 8 blue NECTAR):
 * <table>
 *   <tr><th></th><th>POLLEN</th><th>NECTAR</th></tr>
 *   <tr><td>Diameter</td><td>2.8 in</td><td>3.6 in</td></tr>
 *   <tr><td>Weight</td><td>0.055 lb (25 g)</td><td>0.091 lb (41 g)</td></tr>
 *   <tr><td>Colour</td><td>yellow</td><td>red or blue</td></tr>
 *   <tr><td>Who can score it</td><td>either alliance</td><td>only the matching alliance</td></tr>
 * </table>
 *
 * <h2>Why the shooter must care</h2>
 * NECTAR is 1.65x the mass and 1.29x the diameter of POLLEN. Two separate effects, pulling the
 * same direction:
 * <ol>
 *   <li><b>Mass.</b> For a given flywheel RPM, a heavier ball takes more momentum out of the
 *       wheel during the brief contact, so it leaves slower. More mass also means the wheel sags
 *       more on each shot.</li>
 *   <li><b>Compression.</b> A 3.6 in ball squeezed through a gap sized for a 2.8 in ball is
 *       compressed much harder. That increases grip (good, less slip) but also increases the
 *       energy lost to deforming the ball (bad).</li>
 * </ol>
 * The net effect on almost every FTC shooter is that NECTAR needs noticeably <i>more</i> RPM than
 * POLLEN for the same distance -- which is exactly the behaviour you described wanting. Rather
 * than hard-code "add N rpm for nectar", we express it as a <b>transfer efficiency</b>: what
 * fraction of the flywheel's surface speed the ball actually leaves with. The ballistics solver
 * computes the ball speed physics demands, and this number converts that into an RPM.
 *
 * <p>That split matters because it keeps one tuned number per ball type instead of a whole second
 * distance-to-RPM table.
 */
public enum BallType {

    /** Yellow, 2.8 in, 25 g. Either alliance may score it. */
    POLLEN(2.8, 0.025, 0.50, 0.0),

    /** Red 3.6 in, 41 g. Only the RED alliance may score it. */
    NECTAR_RED(3.6, 0.041, 0.42, 0.0),

    /** Blue 3.6 in, 41 g. Only the BLUE alliance may score it. */
    NECTAR_BLUE(3.6, 0.041, 0.42, 0.0),

    /**
     * Nothing in the hopper, or a colour we could not classify.
     *
     * <p>Efficiency is 0 rather than POLLEN's 0.50 <b>on purpose</b>. If an unclassified ball
     * inherited POLLEN's calibration, a misread NECTAR would be fired with POLLEN's RPM and miss by
     * roughly 8 inches -- and it would look like a ballistics bug, not a sensor bug. A 0 makes
     * {@link #canFire()} false, so the shot is refused instead of taken badly.
     */
    NONE(0, 0, 0.0, 0.0);

    /** Ball diameter, inches. Used by the vision ball tracker to estimate distance. */
    public final double diameterIn;

    /** Mass, kilograms. Informational -- the tuned efficiency already captures its effect. */
    public final double massKg;

    /**
     * Fraction of flywheel surface speed that the ball leaves with. <b>TUNE ME.</b>
     *
     * <p>Rough starting points:
     * <ul>
     *   <li>Single flywheel against a fixed hood: ~0.5 (the contact is rolling, so the ball
     *       leaves at roughly half the surface speed)</li>
     *   <li>Two opposed flywheels: ~0.85-0.95 (both sides drive the ball, almost no slip)</li>
     * </ul>
     * Measure it: command a known RPM, shoot flat off a table, and measure how far the ball flies.
     * {@code T5_ShooterRpmTuner} walks you through it and prints the implied efficiency.
     */
    public final double transferEfficiency;

    /**
     * Air-drag correction, as a fractional speed increase per inch of range. <b>TUNE ME.</b>
     *
     * <p>The solver uses drag-free projectile physics, which always falls SHORT on a light ball --
     * and drag here is not a rounding error. Both BIOBUZZ balls have a terminal velocity of only
     * about 14 m/s, which means the drag-free range prediction is roughly <b>2x optimistic</b>.
     * Leave this at 0 until the pure physics aims consistently, then raise it until long shots stop
     * dropping low. A value of 0.0008 adds 8% speed at 100 inches.
     *
     * <p><b>Tune ONE value and use it for both ball types.</b> An earlier version of this comment
     * reasoned that NECTAR's higher mass-to-area ratio would make it less drag-affected, so its
     * value should end up smaller. Running the numbers, that is wrong in practice: the ballistic
     * coefficients are 6.293 kg/m^2 for POLLEN and 6.243 for NECTAR -- a <b>0.8% difference</b>.
     * NECTAR is both heavier and larger, and the two effects very nearly cancel. Tuning two
     * separate drag values would just be fitting noise, and worse, it would let a drag number
     * quietly absorb an error that actually belongs in {@link #transferEfficiency}.
     */
    public final double dragPerInch;

    BallType(double diameterIn, double massKg, double transferEfficiency, double dragPerInch) {
        this.diameterIn = diameterIn;
        this.massKg = massKg;
        this.transferEfficiency = transferEfficiency;
        this.dragPerInch = dragPerInch;
    }

    public boolean isNectar() {
        return this == NECTAR_RED || this == NECTAR_BLUE;
    }

    /**
     * False for {@link #NONE}. Gate the feeder on this: a ball we could not identify has no valid
     * RPM, so firing it is guaranteed to miss and will be blamed on the ballistics.
     */
    public boolean canFire() {
        return transferEfficiency > 1e-6;
    }

    /**
     * Can our alliance legally score this ball? POLLEN is fair game for anyone; NECTAR only
     * counts for the alliance whose colour it is.
     *
     * <p>Use this to <b>reject</b> the opponent's NECTAR at the intake rather than firing it into
     * our own HIVE. This is the other half of what the colour sensor buys you, and it is easy to
     * forget when you are only thinking about shooter power.
     */
    public boolean isScorableBy(RobotConstants.Alliance us) {
        switch (this) {
            case POLLEN:      return true;
            case NECTAR_RED:  return us == RobotConstants.Alliance.RED;
            case NECTAR_BLUE: return us == RobotConstants.Alliance.BLUE;
            default:          return false;
        }
    }
}
