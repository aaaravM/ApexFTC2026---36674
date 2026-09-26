package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.BallSorter;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.util.Ballistics;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;

/**
 * Puts the whole shot-on-the-move loop together: where is the target, how fast are we going, what
 * ball is loaded, therefore what turret angle and what flywheel RPM.
 *
 * <h2>The data flow, once per loop</h2>
 * <pre>
 *   Odometry     -> robot pose, field velocity, rotation rate
 *   HiveTracker  -> target position in FIELD coordinates (vision, bridged by odometry)
 *   BallSorter   -> which ball is loaded, hence the RPM conversion to use
 *        |
 *        v
 *   Ballistics.solve(...)  ->  turret angle + flywheel RPM + flight time
 *        |
 *        v
 *   Turret.setTargetFieldAngleRad(...)   Shooter.setTargetRpm(...)
 *        |
 *        v
 *   isReadyToFire()  -- gate the feeder on this, never on a timer
 * </pre>
 *
 * <h2>Why the fire gate has four conditions and not one</h2>
 * A shot taken when any one of these is false is a wasted ball, and on a robot that is also driving
 * it is very easy for one of them to be false without the driver noticing:
 * <ul>
 *   <li>a valid ballistic solution exists (we are in range, turret can reach)</li>
 *   <li>the turret has settled on that angle</li>
 *   <li>the flywheel has settled at that RPM</li>
 *   <li>the target estimate is fresh enough to believe</li>
 * </ul>
 * Let the code check them and light one indicator for the driver. That is the difference between a
 * turret that looks impressive in the pit and one that scores in a match.
 */
public class AimController {

    private final Odometry odometry;
    private final HiveTracker hiveTracker;
    private final Turret turret;
    private final Shooter shooter;
    private final BallSorter ballSorter;

    private Ballistics.Solution solution = new Ballistics.Solution();
    private boolean aimingEnabled;
    private BallType assumedBall = BallType.POLLEN;

    public AimController(Odometry odometry, HiveTracker hiveTracker, Turret turret,
                         Shooter shooter, BallSorter ballSorter) {
        this.odometry = odometry;
        this.hiveTracker = hiveTracker;
        this.turret = turret;
        this.shooter = shooter;
        this.ballSorter = ballSorter;
    }

    /**
     * Turn auto-aim on or off. When off, the turret is parked forward and the flywheel is stopped,
     * so the robot is not burning battery spinning a wheel it is not about to use.
     */
    public void setAimingEnabled(boolean enabled) { aimingEnabled = enabled; }

    public boolean isAimingEnabled() { return aimingEnabled; }

    /**
     * Run one iteration. Expects {@link Odometry#update()}, {@link HiveTracker#update} and
     * {@link BallSorter#update()} to have already run this loop.
     */
    public void update() {
        if (!aimingEnabled) {
            turret.setTargetAngleRad(0);
            turret.update(odometry.getHeadingVelRad());
            shooter.stop();
            solution = new Ballistics.Solution();
            solution.failReason = "auto-aim off";
            return;
        }

        // Which ball are we about to fire? If the sensor sees nothing, keep the flywheel spun up
        // for whatever we last had loaded so the next ball does not wait for a spin-up.
        BallType loaded = ballSorter.getBallType();
        if (loaded != BallType.NONE) assumedBall = loaded;

        if (!hiveTracker.hasTarget()) {
            solution = new Ballistics.Solution();
            solution.failReason = String.format("no HIVE target (%.1fs stale)",
                    hiveTracker.getEstimateAgeSec());
            // Hold the last turret angle rather than snapping to zero -- the target is probably
            // about to come back into view roughly where it was.
            turret.update(odometry.getHeadingVelRad());
            return;
        }

        // Muzzle position in field coordinates. The muzzle is offset from the tracking point, and
        // at typical turret offsets that is a couple of degrees of aim at mid range -- worth doing.
        double heading = odometry.getHeadingRad();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double muzzleFieldX = odometry.getX()
                + RobotConstants.Shoot.MUZZLE_X_IN * cos - RobotConstants.Shoot.MUZZLE_Y_IN * sin;
        double muzzleFieldY = odometry.getY()
                + RobotConstants.Shoot.MUZZLE_X_IN * sin + RobotConstants.Shoot.MUZZLE_Y_IN * cos;

        solution = Ballistics.solve(
                muzzleFieldX, muzzleFieldY, RobotConstants.Shoot.MUZZLE_Z_IN,
                hiveTracker.getTargetFieldX(), hiveTracker.getTargetFieldY(),
                hiveTracker.getTargetFieldZ(),
                odometry.getFieldVelX(), odometry.getFieldVelY(),
                heading,
                assumedBall);

        if (solution.valid) {
            turret.setTargetFieldAngleRad(solution.fieldAzimuthRad, heading);
            shooter.setTargetRpm(solution.flywheelRpm);
        } else {
            // Out of range or unreachable: keep the wheel at the last good speed so we are ready
            // the moment the driver moves back into range, but stop chasing an impossible angle.
            shooter.setTargetRpm(shooter.getTargetRpm());
        }

        turret.update(odometry.getHeadingVelRad());
    }

    /** All four gates. Feed a ball only when this is true. */
    public boolean isReadyToFire() {
        return aimingEnabled
                && solution.valid
                && turret.isOnTarget()
                && shooter.isAtSpeed()
                && hiveTracker.hasTarget();
    }

    /** One short line explaining what is blocking the shot, for the driver's telemetry. */
    public String getReadinessText() {
        if (!aimingEnabled) return "AIM OFF";
        if (!hiveTracker.hasTarget()) return "NO TARGET";
        if (!solution.valid) return "NO SOLUTION: " + solution.failReason;
        if (!turret.isOnTarget()) return String.format("TURRET %+.1f deg", turret.getErrorDeg());
        if (!shooter.isAtSpeed()) return String.format("SPINNING %+.0f rpm", shooter.getRpmError());
        return "READY";
    }

    public Ballistics.Solution getSolution() { return solution; }

    public BallType getAssumedBall() { return assumedBall; }

    /** Override the ball type, e.g. while tuning with a hopper the sensor cannot see into. */
    public void setAssumedBall(BallType type) { assumedBall = type; }
}
