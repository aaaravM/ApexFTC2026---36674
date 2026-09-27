package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Read-only view of the robot's pose and velocity, sourced from the Pedro Pathing {@link Follower}.
 *
 * <h2>Use this, not {@link Odometry}, on a Pedro robot</h2>
 * {@link Odometry} <b>owns</b> the Pinpoint: it calls {@code hardwareMap.get(...)}, writes the pod
 * offsets, and calls {@code resetPosAndIMU()} in its constructor. Pedro's {@code PinpointLocalizer}
 * does all of the same things. Running both is not merely redundant, it actively breaks:
 *
 * <ul>
 *   <li>{@code hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")} returns the <b>same cached
 *       device instance</b>, so {@code Odometry}'s constructor would overwrite Pedro's offsets and
 *       then zero the very pose Pedro is following to.</li>
 *   <li>{@code Follower.update()} calls {@code localizer.update()} internally. A second
 *       {@code Odometry.update()} doubles the I2C traffic to one device, halving the effective
 *       odometry rate. Nothing errors -- path correction just goes sluggish and the turret's lead
 *       goes stale, which are both very hard to attribute to their real cause.</li>
 * </ul>
 *
 * So this class touches no hardware for pose. It reads the Follower.
 *
 * <h2>Two things Pedro v3 does not give you, kept here deliberately</h2>
 * <ol>
 *   <li><b>Pose history for vision latency compensation.</b> v2 had {@code
 *       com.pedropathing.util.PoseHistory}; v3 core has no equivalent. A camera frame is 30-100 ms
 *       old by the time it is processed, which at 60 in/s is up to 6 inches of stale position, so
 *       {@link #getPoseAt(long)} is still needed and is still ours to maintain.</li>
 *   <li><b>Pinpoint fault reporting.</b> v3 has none. A disconnected pod is the most common odometry
 *       failure and the device reports it, so {@link #getStatusText()} is worth keeping. Reading
 *       {@code getDeviceStatus()}, {@code getLoopTime()} and {@code getFrequency()} from a separate
 *       driver handle is <b>safe</b> -- those accessors return cached register values and do not
 *       trigger an I2C {@code update()}, so they cannot race Pedro.</li>
 * </ol>
 *
 * <h2>The frame trap</h2>
 * {@code follower.velocity()} is <b>FIELD</b> frame; {@code follower.twist()} is <b>ROBOT</b> frame.
 * Both are {@code (vx, vy, omega)} with identical field names and an identical {@code toString}, so
 * mixing them up is invisible on inspection -- and it produces exactly the symptom
 * {@code T3_VelocityFrameTest} was written to catch: the turret leads correctly when you drive along
 * one axis and backwards along the other. Ballistics wants field velocity. This class labels both
 * explicitly so the choice is always visible at the call site.
 */
public class PedroOdometry {

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

    private final Follower follower;
    private final GoBildaPinpointDriver pinpointForDiagnosticsOnly;
    private final Deque<TimedPose> history = new ArrayDeque<>();

    private double x, y, headingRad;
    private double fieldVelX, fieldVelY, headingVelRad;
    private double robotVelForward, robotVelLeft;

    public PedroOdometry(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        // Diagnostics only. Never call update() or any setter on this handle -- it is the same
        // object Pedro's localizer holds.
        GoBildaPinpointDriver handle = null;
        try {
            handle = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.Names.PINPOINT);
        } catch (Exception ignored) {
            // Diagnostics are a nicety; never let them stop an OpMode from running.
        }
        this.pinpointForDiagnosticsOnly = handle;
    }

    /**
     * Sample the Follower. Call once per loop, <b>after</b> {@code follower.update()} -- that is what
     * refreshes the localizer, so sampling before it gives you last loop's numbers.
     */
    public void update() {
        Pose pose = follower.pose();
        x = pose.x();
        y = pose.y();
        // Pedro normalises heading to [0, 2*pi). Everything else in this codebase -- Ballistics,
        // Turret, the wrapRad helper -- assumes [-pi, pi]. Convert once, here, rather than
        // discovering the discontinuity as a turret that flips 360 degrees near zero heading.
        headingRad = wrap(pose.heading());

        Velocity velocity = follower.velocity();      // FIELD frame
        fieldVelX = velocity.vx;
        fieldVelY = velocity.vy;
        headingVelRad = velocity.omega;               // identical in both frames

        com.pedropathing.math.Twist twist = follower.twist();   // ROBOT frame
        robotVelForward = twist.vx;
        robotVelLeft = twist.vy;

        long now = System.nanoTime();
        history.addLast(new TimedPose(now, x, y, headingRad));
        long cutoff = now - (long) (RobotConstants.Odo.POSE_HISTORY_SECONDS * 1e9);
        while (!history.isEmpty() && history.peekFirst().nanoTime < cutoff) {
            history.removeFirst();
        }
    }

    // ---------------------------------------------------------------------------------------
    // Pose
    // ---------------------------------------------------------------------------------------

    /** Field X, inches. */
    public double getX() { return x; }

    /** Field Y, inches. */
    public double getY() { return y; }

    /** Heading, radians CCW, wrapped to [-pi, pi] -- NOT Pedro's native [0, 2*pi). */
    public double getHeadingRad() { return headingRad; }

    public double getHeadingDeg() { return Math.toDegrees(headingRad); }

    /** Pedro's raw heading, [0, 2*pi). Only for telemetry that should match Pedro's own output. */
    public double getPedroHeadingRad() { return follower.pose().heading(); }

    // ---------------------------------------------------------------------------------------
    // Velocity
    // ---------------------------------------------------------------------------------------

    /** FIELD-frame X velocity, in/s. This is what the ballistics solver wants. */
    public double getFieldVelX() { return fieldVelX; }

    /** FIELD-frame Y velocity, in/s. This is what the ballistics solver wants. */
    public double getFieldVelY() { return fieldVelY; }

    /** ROBOT-frame forward velocity, in/s. For drivetrain feedforward, not for ballistics. */
    public double getRobotVelForward() { return robotVelForward; }

    /** ROBOT-frame leftward velocity, in/s. */
    public double getRobotVelLeft() { return robotVelLeft; }

    /** Speed regardless of direction, in/s. Frame-independent, so always safe. */
    public double getSpeed() { return Math.hypot(fieldVelX, fieldVelY); }

    /** Rotation rate, rad/s CCW. The turret uses this as feedforward. */
    public double getHeadingVelRad() { return headingVelRad; }

    // ---------------------------------------------------------------------------------------
    // Latency compensation -- no library equivalent in Pedro v3
    // ---------------------------------------------------------------------------------------

    /**
     * The pose at a past instant, linearly interpolated from history. Vision passes
     * {@code detection.frameAcquisitionNanoTime} here so a sighting is combined with the pose that
     * was true when the shutter opened rather than the pose now.
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
                // Interpolate heading the short way round so we never blend across the seam.
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
    // Diagnostics -- no library equivalent in Pedro v3
    // ---------------------------------------------------------------------------------------

    /**
     * Pinpoint health as a telemetry string. Check this FIRST when odometry misbehaves: a
     * {@code FAULT_X_POD_NOT_DETECTED} is a cable, not a constant.
     */
    public String getStatusText() {
        if (pinpointForDiagnosticsOnly == null) return "no diagnostics handle";
        GoBildaPinpointDriver.DeviceStatus status = pinpointForDiagnosticsOnly.getDeviceStatus();
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
        return pinpointForDiagnosticsOnly != null
                && pinpointForDiagnosticsOnly.getDeviceStatus()
                   == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /** Pinpoint internal loop time in microseconds. Should sit near 650 us (~1500 Hz). */
    public int getPinpointLoopTimeUs() {
        return pinpointForDiagnosticsOnly == null ? -1 : pinpointForDiagnosticsOnly.getLoopTime();
    }

    /** Pinpoint update frequency, Hz. */
    public double getPinpointFrequency() {
        return pinpointForDiagnosticsOnly == null ? -1 : pinpointForDiagnosticsOnly.getFrequency();
    }

    private static double wrap(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
