package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Wraps the goBILDA Pinpoint Odometry Computer (4-Bar Odometry Pack, part 3110-0002-0001).
 *
 * <h2>Why this class exists instead of using the driver directly</h2>
 * <ol>
 *   <li><b>One place for units.</b> The Pinpoint speaks millimetres and radians. Every getter here
 *       returns inches and radians, asked for explicitly, so a unit mix-up cannot happen silently.</li>
 *   <li><b>Field velocity.</b> Shooting on the move needs the robot's velocity in the FIELD frame.
 *       See {@link #getFieldVelX()} for the one flag you must verify on your robot.</li>
 *   <li><b>Pose history.</b> A camera frame describes where the robot was when the shutter opened,
 *       not where it is now. At 3 m/s with 80 ms of latency that is 9 inches of error. We keep half
 *       a second of pose history so vision can ask "where was I at time T?".</li>
 *   <li><b>Fault reporting.</b> A disconnected pod is the single most common odometry failure and
 *       the Pinpoint reports it. {@link #getStatusText()} surfaces it on telemetry instead of
 *       letting you chase a phantom tuning problem.</li>
 * </ol>
 *
 * <h2>Hardware notes that will save you an evening</h2>
 * <ul>
 *   <li>The Pinpoint mounts <b>sticker side up</b>. Upside down inverts the IMU heading.</li>
 *   <li>Put it on <b>I2C bus 1, 2 or 3 -- never bus 0</b>. Bus 0 is shared with the Control Hub's
 *       internal IMU and the contention shows up as intermittent read faults.</li>
 *   <li>Configure it in the Robot Configuration as <i>goBILDA Pinpoint Odometry Computer</i>.
 *       As of SDK 12.0 the driver ships with the SDK -- you do not copy any file into TeamCode.</li>
 *   <li>The robot must be <b>completely still</b> when {@link #resetPoseAndCalibrateImu()} runs.
 *       A robot that is being carried during init gets a bad IMU bias for the whole match.</li>
 * </ul>
 */
public class Odometry {

    /** A pose plus the timestamp it was true at. */
    public static final class TimedPose {
        public final long nanoTime;
        public final double x, y, headingRad;
        TimedPose(long nanoTime, double x, double y, double headingRad) {
            this.nanoTime = nanoTime;
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }
    }

    private final GoBildaPinpointDriver pinpoint;
    private final Deque<TimedPose> history = new ArrayDeque<>();

    private double x, y, headingRad;
    private double rawVelX, rawVelY, headingVelRad;
    private long lastUpdateNano;

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.Names.PINPOINT);

        // Where the pods are, relative to the point we want the pose to describe.
        pinpoint.setOffsets(RobotConstants.Odo.X_POD_OFFSET_MM,
                            RobotConstants.Odo.Y_POD_OFFSET_MM,
                            DistanceUnit.MM);

        // The 4-Bar pod is 19.89436789 ticks/mm. Naming the enum instead of the number means the
        // SDK stays the source of truth if goBILDA ever revises it.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                RobotConstants.Odo.REVERSE_X_ENCODER
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RobotConstants.Odo.REVERSE_Y_ENCODER
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);

        resetPoseAndCalibrateImu();
    }

    /** Zero the pose and recalibrate the IMU. The robot MUST be stationary. */
    public void resetPoseAndCalibrateImu() {
        pinpoint.resetPosAndIMU();
        history.clear();
        x = y = headingRad = 0;
        rawVelX = rawVelY = headingVelRad = 0;
    }

    /**
     * Read the Pinpoint. Call this <b>exactly once near the top of every loop</b> -- it is the only
     * thing that performs the I2C transaction, and calling it twice per loop halves your loop rate
     * for no benefit.
     */
    public void update() {
        pinpoint.update();
        lastUpdateNano = System.nanoTime();

        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        headingRad = pinpoint.getHeading(AngleUnit.RADIANS);

        rawVelX = pinpoint.getVelX(DistanceUnit.INCH);
        rawVelY = pinpoint.getVelY(DistanceUnit.INCH);
        headingVelRad = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        history.addLast(new TimedPose(lastUpdateNano, x, y, headingRad));
        long cutoff = lastUpdateNano - (long) (RobotConstants.Odo.POSE_HISTORY_SECONDS * 1e9);
        while (!history.isEmpty() && history.peekFirst().nanoTime < cutoff) {
            history.removeFirst();
        }
    }

    // ---------------------------------------------------------------------------------------
    // Pose
    // ---------------------------------------------------------------------------------------

    /** Field X, inches. +X is the direction the robot faced when the pose was last zeroed. */
    public double getX() { return x; }

    /** Field Y, inches. +Y is to the left of that direction. */
    public double getY() { return y; }

    /** Heading, radians CCW, wrapped to [-pi, pi]. */
    public double getHeadingRad() { return headingRad; }

    public double getHeadingDeg() { return Math.toDegrees(headingRad); }

    /** Set the pose, e.g. to your known starting position at the start of autonomous. */
    public void setPose(double xIn, double yIn, double headingRadians) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn,
                AngleUnit.RADIANS, headingRadians));
        history.clear();
    }

    // ---------------------------------------------------------------------------------------
    // Velocity -- the part that makes shooting on the move possible
    // ---------------------------------------------------------------------------------------

    /**
     * Robot velocity along the FIELD +X axis, inches/sec.
     *
     * <p><b>Read this before you trust it.</b> goBILDA's driver documents these as
     * "X (forward) velocity" and "Y (strafe) velocity", which reads as robot-relative -- but it
     * documents <i>position</i> the same way, and position is unambiguously a field coordinate.
     * The naming is inherited from the pod names and is not reliable evidence either way.
     *
     * <p>So we do not guess. {@code T3_VelocityFrameTest} determines it empirically on your robot:
     * drive straight forward while holding a 45-degree heading, and see whether the reported
     * velocity splits between both axes (field frame) or stays on one axis (robot frame). Set
     * {@link RobotConstants.Odo#VELOCITY_IS_ROBOT_RELATIVE} to match and this method is correct
     * either way.
     *
     * <p>Getting this wrong does not look like a crash. It looks like the turret leading correctly
     * when you drive along one axis and leading the wrong way when you drive along the other --
     * which is a genuinely horrible thing to debug from the driver's seat. Spend the 20 seconds.
     */
    public double getFieldVelX() {
        if (!RobotConstants.Odo.VELOCITY_IS_ROBOT_RELATIVE) return rawVelX;
        return rawVelX * Math.cos(headingRad) - rawVelY * Math.sin(headingRad);
    }

    /** Robot velocity along the FIELD +Y axis, inches/sec. See {@link #getFieldVelX()}. */
    public double getFieldVelY() {
        if (!RobotConstants.Odo.VELOCITY_IS_ROBOT_RELATIVE) return rawVelY;
        return rawVelX * Math.sin(headingRad) + rawVelY * Math.cos(headingRad);
    }

    /** Forward velocity in the ROBOT frame, inches/sec. Handy for drivetrain feedforward. */
    public double getRobotVelForward() {
        if (RobotConstants.Odo.VELOCITY_IS_ROBOT_RELATIVE) return rawVelX;
        return rawVelX * Math.cos(headingRad) + rawVelY * Math.sin(headingRad);
    }

    /** Leftward velocity in the ROBOT frame, inches/sec. */
    public double getRobotVelLeft() {
        if (RobotConstants.Odo.VELOCITY_IS_ROBOT_RELATIVE) return rawVelY;
        return -rawVelX * Math.sin(headingRad) + rawVelY * Math.cos(headingRad);
    }

    /** Speed regardless of direction, inches/sec. Frame-independent, so always safe. */
    public double getSpeed() { return Math.hypot(rawVelX, rawVelY); }

    /** Rotation rate, radians/sec CCW. The turret uses this as feedforward. */
    public double getHeadingVelRad() { return headingVelRad; }

    /** Unrotated Pinpoint velocity, exactly as reported. Only the frame test should need these. */
    public double getRawVelX() { return rawVelX; }
    public double getRawVelY() { return rawVelY; }

    // ---------------------------------------------------------------------------------------
    // Latency compensation
    // ---------------------------------------------------------------------------------------

    /**
     * The pose at a past instant, linearly interpolated from history. Vision passes
     * {@code detection.frameAcquisitionNanoTime} here so the sighting is combined with the pose
     * that was true when the shutter opened rather than the pose now.
     *
     * @param nanoTime a {@link System#nanoTime()} value
     * @return the interpolated pose, or the oldest/newest we have if the time is out of range
     */
    public TimedPose getPoseAt(long nanoTime) {
        if (history.isEmpty()) return new TimedPose(nanoTime, x, y, headingRad);
        if (nanoTime >= history.peekLast().nanoTime) return history.peekLast();
        if (nanoTime <= history.peekFirst().nanoTime) return history.peekFirst();

        TimedPose previous = history.peekFirst();
        for (TimedPose p : history) {
            if (p.nanoTime >= nanoTime) {
                double span = p.nanoTime - previous.nanoTime;
                double fraction = span <= 0 ? 0 : (nanoTime - previous.nanoTime) / span;
                // Interpolate heading the short way round so we never blend across the +/-pi seam.
                double dHeading = wrap(p.headingRad - previous.headingRad);
                return new TimedPose(nanoTime,
                        previous.x + (p.x - previous.x) * fraction,
                        previous.y + (p.y - previous.y) * fraction,
                        wrap(previous.headingRad + dHeading * fraction));
            }
            previous = p;
        }
        return history.peekLast();
    }

    // ---------------------------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------------------------

    /**
     * Pinpoint health, as a string fit for telemetry. Check this FIRST whenever odometry misbehaves.
     * A {@code FAULT_X_POD_NOT_DETECTED} means a cable, not a constant.
     */
    public String getStatusText() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        switch (status) {
            case READY:                     return "READY";
            case CALIBRATING:               return "CALIBRATING (hold still)";
            case NOT_READY:                 return "NOT_READY (still booting)";
            case FAULT_X_POD_NOT_DETECTED:  return "FAULT: X pod not detected -- check its cable";
            case FAULT_Y_POD_NOT_DETECTED:  return "FAULT: Y pod not detected -- check its cable";
            case FAULT_NO_PODS_DETECTED:    return "FAULT: no pods -- check both cables";
            case FAULT_IMU_RUNAWAY:         return "FAULT: IMU runaway -- recalibrate while still";
            case FAULT_BAD_READ:            return "FAULT: bad I2C read -- is it on bus 0? move it";
            default:                        return status.toString();
        }
    }

    public boolean isHealthy() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /** Pinpoint's own internal loop time in microseconds. Should sit around 650 us (~1500 Hz). */
    public int getPinpointLoopTimeUs() { return pinpoint.getLoopTime(); }

    /** Pinpoint update frequency in Hz. */
    public double getPinpointFrequency() { return pinpoint.getFrequency(); }

    /** Raw pod ticks. Non-zero movement here with a frozen pose means a configuration problem. */
    public int getEncoderX() { return pinpoint.getEncoderX(); }
    public int getEncoderY() { return pinpoint.getEncoderY(); }

    /** Direct access, for the rare thing this wrapper does not expose. Prefer the wrapper. */
    public GoBildaPinpointDriver raw() { return pinpoint; }

    private static double wrap(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
