package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * Pedro Pathing's Foresight (path-following) tuning numbers.
 *
 * <h2>Why this is its own class, with no Android imports</h2>
 * Deliberately. Everything here comes from Pedro's pure-Java {@code core} jar, so this class can be
 * compiled and <b>executed on a laptop</b> without a robot, an Android SDK, or an OpMode. That is how
 * the "all twelve required fields are set and every validator accepts them" claim below was actually
 * checked rather than hoped for.
 *
 * <h2>ForesightConfig has exactly 12 required fields</h2>
 * This is the nastiest failure mode in a Pedro v3 setup: {@code ForesightConfig} has 35 fields, 23 of
 * which carry defaults and <b>12 of which do not</b>. A config missing any required field
 * <b>compiles cleanly</b> and then throws
 * {@code IllegalStateException("Config variable has not been set")} on the first
 * {@code follower.update()} -- so the OpMode dies the instant you press START, with a message that
 * does not name the missing field.
 *
 * <p>The 12, determined by reflecting on {@code ConfigVar.hasValue} in an empty config rather than by
 * reading docs:
 * <ol>
 *   <li>{@code headingFeedback} (Controller)</li>
 *   <li>{@code forwardTranslational} (Controller)</li>
 *   <li>{@code strafeTranslational} (Controller)</li>
 *   <li>{@code brake} (Controller)</li>
 *   <li>{@code coast} (Controller)</li>
 *   <li>{@code linearBrakeCoefficients} (Matrix)</li>
 *   <li>{@code quadraticBrakeCoefficients} (Matrix)</li>
 *   <li>{@code headingBrakeCoefficients} (Vector2D)</li>
 *   <li>{@code maxAchievableForwardVelocity} (Double)</li>
 *   <li>{@code maxAchievableStrafeVelocity} (Double)</li>
 *   <li>{@code naturalForwardDeceleration} (Double)</li>
 *   <li>{@code naturalStrafeDeceleration} (Double)</li>
 * </ol>
 * All 12 are set below, so the "not been set" exception cannot happen.
 *
 * <h2>These are PLACEHOLDERS, and that is fine for now</h2>
 * They are structurally valid and they let the robot drive, but they are not tuned for your robot.
 * Replace them with AutoTune's output.
 *
 * <p><b>None of these numbers affect teleop driving.</b> {@code follower.manual(...)} does not run
 * the path-following algorithm, so you can drive all day on placeholders. They become load-bearing
 * only for autonomous paths, {@code follower.hold(...)}, and
 * {@code ManualDrive.driveOrHold(...)}.
 *
 * <h2>The v2 rename that will trip you up</h2>
 * <table>
 *   <tr><th>v2 name</th><th>v3 name</th></tr>
 *   <tr><td>{@code xVelocity}</td><td>{@code maxAchievableForwardVelocity}</td></tr>
 *   <tr><td>{@code yVelocity}</td><td>{@code maxAchievableStrafeVelocity}</td></tr>
 *   <tr><td>{@code forwardZeroPowerAcceleration}</td><td>{@code naturalForwardDeceleration}</td></tr>
 *   <tr><td>{@code lateralZeroPowerAcceleration}</td><td>{@code naturalStrafeDeceleration}</td></tr>
 * </table>
 * v2's zero-power accelerations were <b>negative</b>. v3's natural decelerations are <b>positive
 * magnitudes</b>. Pasting a v2 number in throws at construction -- which is the good case, because it
 * fails loudly instead of quietly driving badly.
 */
public final class FollowerTuning {

    private FollowerTuning() {}

    // ---- Measured by Pedro's ForesightTuner. Placeholders until you run it. -------------------
    //
    // Sanity bounds for this robot: 4x goBILDA 5203 19.2:1 (312 RPM) on 104 mm mecanum wheels gives
    // a THEORETICAL free speed of 66.9 in/s. A real mecanum robot reaches 80-85% of that forward and
    // about 0.8x that sideways, so expect roughly 53-58 and 42-46 in/s measured. If AutoTune reports
    // a forward velocity above ~67 in/s, something is wrong with the measurement, not the robot.

    /** in/s. */
    public static final double MAX_FORWARD_VELOCITY = 55.0;
    /** in/s. */
    public static final double MAX_STRAFE_VELOCITY = 44.0;
    /** in/s^2, POSITIVE magnitude (v3 convention). */
    public static final double NATURAL_FORWARD_DECEL = 60.0;
    /** in/s^2, POSITIVE magnitude (v3 convention). */
    public static final double NATURAL_STRAFE_DECEL = 70.0;

    // ---- Controller gains. Placeholders. -----------------------------------------------------

    /** Heading error (radians) -> turn power. */
    public static final double HEADING_KP = 1.0, HEADING_KI = 0.0, HEADING_KD = 0.05;
    /** Along-path position error (inches) -> drive power. */
    public static final double FORWARD_KP = 0.10, FORWARD_KI = 0.0, FORWARD_KD = 0.01;
    /** Cross-track position error (inches) -> strafe power. */
    public static final double STRAFE_KP = 0.10, STRAFE_KI = 0.0, STRAFE_KD = 0.01;

    /** Deceleration authority near the end of a path. */
    public static final double BRAKE_KP = 0.02;
    /** Gentle correction while coasting in. */
    public static final double COAST_KP = 0.01;

    /**
     * Every required field, set. Pure core types only -- no Android, so this is testable off-robot.
     */
    public static ForesightConfig build() {
        return new ForesightConfig(c -> {
            // AutoTuned 2026-10-03 (24" runs). Verified after the 10-11 rewire with Square Auto.
            Controller primaryTranslationalForward   = Controller.proportional(0.19413400497965824);
            Controller secondaryTranslationalForward = Controller.proportional(0.07172732748383374);
            Controller primaryTranslationalLateral   = Controller.proportional(0.23764279539719071);
            Controller secondaryTranslationalLateral = Controller.proportional(0.08780266296682057);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.017389356677845302));
            c.brake.set(Controller.proportionalFeedforward(0.014780953176168506));

            c.headingFeedback.set(Controller.proportional(3.04943011525379));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.044347804237630274, 0.005030203878913554));

            c.linearBrakeCoefficients.set(Matrix.diag(0.08660325064525407, 0.06663123813222216));
            c.quadraticBrakeCoefficients.set(Matrix.diag(5.83618534644232E-4, 8.069577156433441E-4));

            c.maxAchievableForwardVelocity.set(58.424670577668905);
            c.maxAchievableStrafeVelocity.set(49.014295988583314);
            c.naturalForwardDeceleration.set(30.551663298245547);
            c.naturalStrafeDeceleration.set(59.38720787324426);
        });
    }
}
