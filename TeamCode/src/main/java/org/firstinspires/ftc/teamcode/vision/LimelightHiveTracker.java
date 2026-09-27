package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.PedroOdometry;

import java.util.List;

/**
 * Tracks the HIVE CELL with a <b>Limelight 3A</b>, and keeps a field-frame estimate of where it is.
 *
 * <h2>Why this exists instead of {@link HiveTracker}</h2>
 * {@link HiveTracker} uses the SDK's {@code VisionPortal} + {@code AprilTagProcessor} +
 * {@code AprilTagClusterDetection}. That whole pipeline runs <b>on the Control Hub</b> and needs a UVC
 * webcam: {@code VisionPortal} consumes a {@code CameraName} and processes frames itself.
 *
 * <p>A Limelight 3A is a separate coprocessor. {@code Limelight3A} implements {@code HardwareDevice}
 * and returns an already-parsed {@code LLResult} over USB-ethernet. It is <b>not</b> a
 * {@code WebcamName}, not a {@code CameraName}, and there is no adapter in either direction. So:
 *
 * <ul>
 *   <li>{@code HiveTracker} cannot run on a Limelight-only robot -- it throws at init.</li>
 *   <li>A Limelight can never emit an {@code AprilTagClusterDetection}. The SDK's BIOBUZZ cluster
 *       solver is simply not available to us.</li>
 * </ul>
 *
 * <h2>What we lose, and why it is almost nothing</h2>
 * When only one member tag is visible, the SDK's "cluster solve" is just single-tag pose estimation
 * plus a rigid offset. That is exactly what this class does, using the offsets in
 * {@link BiobuzzTags} extracted from the SDK binary. With more tags visible we average them, which
 * recovers most of the stability benefit -- and because the four x offsets are symmetric and sum to
 * zero, seeing all four gives the origin's sideways position <b>exactly</b>, with no offset maths.
 *
 * <h2>What we gain</h2>
 * The Limelight does its own processing at 640x480 and a high frame rate for <b>zero Control Hub CPU
 * cost</b>, with lower latency than on-Hub AprilTag detection. On this robot the Hub is already
 * running Pedro Pathing, Pinpoint odometry, the shooter velocity loop and the turret loop, so moving
 * the latency-critical vision job onto a coprocessor is the right allocation.
 *
 * <h2>Three traps this class handles for you</h2>
 * <ol>
 *   <li><b>Sign reversal.</b> Limelight {@code tx} is positive when the target is to the <b>RIGHT</b>.
 *       FTC's {@code AprilTagPoseFtc.bearing} -- and every sign convention in this codebase -- treats
 *       positive as <b>LEFT</b>. We negate it. Get this wrong and the turret slews <i>away</i> from
 *       the HIVE, which presents as a turret-controller bug.</li>
 *   <li><b>Units.</b> Every {@code Pose3D} from an {@code LLResult} is in <b>metres and degrees</b>.
 *       This whole codebase is inches. Converted once, here, at the boundary.</li>
 *   <li><b>The timestamp domain.</b> {@code getControlHubTimeStampNanos()} looks like the right thing
 *       to feed {@link PedroOdometry#getPoseAt(long)}, and it is not: it is
 *       {@code currentTimeMillis() * 1_000_000}, i.e. wall-clock epoch, while {@code getPoseAt}
 *       expects {@code System.nanoTime()}. Mixing them compiles, throws nothing, and returns poses
 *       wrong by the machine's uptime-vs-epoch difference -- hours. We stay in the nanoTime domain by
 *       subtracting measured latency from {@code System.nanoTime()} instead.</li>
 * </ol>
 */
public class LimelightHiveTracker {

    /** Pipeline index configured for AprilTag/fiducial detection in the Limelight web UI. */
    public static final int APRILTAG_PIPELINE = 0;

    private final Limelight3A limelight;

    // Filtered field-frame estimate of the aim point.
    private double targetFieldX, targetFieldY, targetFieldZ;
    private boolean hasEstimate;
    private long lastSightingNano;

    // Most recent raw sighting, for telemetry.
    private double lastRangeIn, lastBearingDeg, lastElevationDeg;
    private int lastTagCount;
    private String lastClusterName = "none";
    private double lastLatencyMs;
    private int fiducialsSeen;

    public LimelightHiveTracker(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Names.LIMELIGHT);

        // setPollRateHz silently does NOTHING if the poller is already RUNNING -- no exception, no
        // log. So it has to happen before start().
        limelight.setPollRateHz(RobotConstants.Camera.LIMELIGHT_POLL_HZ);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    /**
     * Process the newest result and refresh the field-frame target estimate.
     *
     * @param odometry       already updated this loop
     * @param turretAngleRad current turret angle; used when the Limelight rides on the turret
     */
    public void update(PedroOdometry odometry, double turretAngleRad) {
        LLResult result = limelight.getLatestResult();
        // getLatestResult() has not returned null since SDK 10.3 -- the SDK sample's own comment
        // claiming otherwise is stale -- but isValid() still has to be checked.
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        fiducialsSeen = fiducials == null ? 0 : fiducials.size();
        if (fiducials == null || fiducials.isEmpty()) return;

        // Accumulate only the tags belonging to OUR alliance's CELL. Without this filter the turret
        // will happily aim at the opponent's HIVE, which is a legal shot that scores them nothing
        // and us nothing.
        double sumForward = 0, sumLeft = 0, sumUp = 0, sumXOffset = 0;
        int count = 0;
        BiobuzzTags.Cluster cluster = null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            BiobuzzTags.Cluster c = BiobuzzTags.clusterOf(id);
            if (c == null) continue;                       // not a CELL tag
            if (c.alliance != RobotConstants.alliance) continue;   // opponent's HIVE
            if (cluster != null && c != cluster) continue;         // stay locked to one CELL
            cluster = c;

            // Tag pose in CAMERA space. Pose3D is metres + degrees, so convert immediately.
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);

            // Limelight camera space: +x right, +y down, +z forward (out of the lens).
            // Convert to this codebase's convention: +forward, +left, +up.
            sumForward += p.z;
            sumLeft += -p.x;
            sumUp += -p.y;
            sumXOffset += BiobuzzTags.memberXOffsetIn(id);
            count++;
        }

        if (count == 0) return;

        double forward = sumForward / count;
        double left = sumLeft / count;
        double up = sumUp / count;
        double meanXOffset = sumXOffset / count;

        // ---- Tag centroid -> CELL opening centre ------------------------------------------------
        // The member x offsets are symmetric and sum to zero, so with all four tags visible
        // meanXOffset is 0 and no sideways correction is needed at all. With a subset, subtract the
        // mean offset of the tags we actually saw.
        //
        // The y/z offsets are the same for every member, so that part is a constant.
        //
        // These are applied in the ROBOT frame rather than rotated out of the tag frame. That is
        // exact when looking at the HIVE roughly square-on and degrades gently as you view it from
        // an angle. It avoids building a rotation matrix from yaw/pitch/roll, which is a large
        // sign-convention risk for a correction of only a few inches. The three constants are
        // tunable so you can dial the residual out empirically with T4.
        left -= meanXOffset * RobotConstants.Hive.CELL_OFFSET_SCALE;
        forward += RobotConstants.Hive.CELL_OFFSET_FORWARD_IN;
        up += RobotConstants.Hive.CELL_OFFSET_UP_IN;

        double horizontalRange = Math.hypot(forward, left);
        lastRangeIn = Math.sqrt(forward * forward + left * left + up * up);
        // NEGATED relative to Limelight's own tx: we report positive = LEFT.
        lastBearingDeg = Math.toDegrees(Math.atan2(left, forward));
        lastElevationDeg = Math.toDegrees(Math.atan2(up, horizontalRange));
        lastTagCount = count;
        lastClusterName = cluster.shortName;

        // ---- Camera frame -> robot frame --------------------------------------------------------
        double cameraYaw = Math.toRadians(RobotConstants.Camera.YAW_DEG);
        if (RobotConstants.Camera.MOUNTED_ON_TURRET) cameraYaw += turretAngleRad;

        double cos = Math.cos(cameraYaw);
        double sin = Math.sin(cameraYaw);
        double robotX = RobotConstants.Camera.X_IN + forward * cos - left * sin;
        double robotY = RobotConstants.Camera.Y_IN + forward * sin + left * cos;
        double robotZ = RobotConstants.Camera.Z_IN + up;

        // ---- Robot frame -> field frame, using the pose from when the frame was captured --------
        // Total age of this result in the System.nanoTime() domain. getStaleness() is how long ago
        // the LLResult was built on the Hub; capture + targeting latency is the time spent on the
        // camera before that.
        lastLatencyMs = result.getStaleness()
                + result.getCaptureLatency()
                + result.getTargetingLatency();
        long frameNano = System.nanoTime() - (long) (lastLatencyMs * 1e6);

        PedroOdometry.TimedPose then = odometry.getPoseAt(frameNano);
        double hCos = Math.cos(then.headingRad);
        double hSin = Math.sin(then.headingRad);
        double fieldX = then.x + robotX * hCos - robotY * hSin;
        double fieldY = then.y + robotX * hSin + robotY * hCos;
        double fieldZ = robotZ + RobotConstants.Hive.AIM_Z_BIAS_IN;

        // ---- Blend into the running estimate ---------------------------------------------------
        if (!hasEstimate) {
            targetFieldX = fieldX;
            targetFieldY = fieldY;
            targetFieldZ = fieldZ;
            hasEstimate = true;
        } else {
            double a = RobotConstants.Hive.POSITION_FILTER_ALPHA;
            targetFieldX += a * (fieldX - targetFieldX);
            targetFieldY += a * (fieldY - targetFieldY);
            targetFieldZ += a * (fieldZ - targetFieldZ);
        }
        lastSightingNano = System.nanoTime();
    }

    /** True if the estimate is recent enough to shoot at. Odometry carries it through gaps. */
    public boolean hasTarget() {
        return hasEstimate && getEstimateAgeSec() <= RobotConstants.Hive.MAX_ESTIMATE_AGE_SEC;
    }

    public double getEstimateAgeSec() {
        if (!hasEstimate) return Double.POSITIVE_INFINITY;
        return (System.nanoTime() - lastSightingNano) / 1e9;
    }

    /** True if the camera is seeing the CELL right now, rather than dead-reckoning it. */
    public boolean isSightingLive() { return getEstimateAgeSec() < 0.25; }

    public double getTargetFieldX() { return targetFieldX; }
    public double getTargetFieldY() { return targetFieldY; }
    public double getTargetFieldZ() { return targetFieldZ; }

    /** Throw away the estimate. Do this when the HIVE tips. */
    public void resetEstimate() { hasEstimate = false; }

    // ---- Telemetry ---------------------------------------------------------------------------

    /** Free local field reads. Use these per-loop, NOT {@code getStatus()}. */
    public boolean isConnected() { return limelight.isConnected(); }
    public boolean isRunning() { return limelight.isRunning(); }

    public double getLastRangeIn() { return lastRangeIn; }
    /** Positive means the CELL is to our LEFT -- already negated from Limelight's convention. */
    public double getLastBearingDeg() { return lastBearingDeg; }
    public double getLastElevationDeg() { return lastElevationDeg; }
    public int getLastTagCount() { return lastTagCount; }
    public String getLastClusterName() { return lastClusterName; }
    public double getLastLatencyMs() { return lastLatencyMs; }
    public int getFiducialsSeen() { return fiducialsSeen; }

    public void stop() { limelight.stop(); }

    /**
     * Direct access. Note that {@code getStatus()} performs a <b>blocking HTTP GET</b> on the calling
     * thread with 100 ms connect and 100 ms read timeouts, so calling it every loop costs up to
     * 200 ms per iteration on a flaky link. Cache it at about 1 Hz. The same applies to
     * {@code pipelineSwitch}, {@code reloadPipeline}, {@code updateRobotOrientation} and all the
     * {@code upload*} methods -- never call those in a loop.
     */
    public Limelight3A raw() { return limelight; }
}
