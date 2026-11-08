package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Tracks the HIVE CELL we want to shoot into, and keeps a field-frame estimate of where it is.
 *
 * <h2>Read this before designing around it: the BIOBUZZ target MOVES</h2>
 * The SDK 12.0 release notes are blunt about it -- <i>"since BIOBUZZ AprilTags move, they are not
 * suitable for absolute Field Localization."</i> The HIVE is bi-stable and TIPS when enough POLLEN
 * or NECTAR lands in the upward-facing CELL, which physically relocates the tags.
 *
 * <p>That inverts the usual FTC architecture, and it is worth being explicit about because the
 * obvious plan is the wrong one:
 * <ul>
 *   <li><b>Do not</b> use the tags to work out where the robot is, then compute the angle to a
 *       hard-coded goal coordinate. The goal coordinate does not exist -- the goal moves.</li>
 *   <li><b>Do</b> use the tags to measure where the target is right now, relative to us, and use
 *       odometry for the robot's own motion. Vision answers "where is the CELL", odometry answers
 *       "how fast am I going and which way am I facing".</li>
 * </ul>
 * One nice consequence: FIRST placed each cluster's origin at the <b>centre of the CELL opening</b>,
 * specifically so it can be aimed at directly. The pose you get back is already the aim point.
 *
 * <h2>What odometry adds on top of vision</h2>
 * Two things vision cannot do by itself:
 * <ol>
 *   <li><b>Bridging dropouts.</b> Each sighting is converted into a FIELD coordinate using the pose
 *       at that instant, then stored. When the camera loses the tags -- the turret slews, a robot
 *       drives through the view, the tag leaves the frame -- we keep converting that stored field
 *       coordinate back into a fresh robot-relative aim using current odometry. The turret keeps
 *       tracking through gaps of a second or more instead of giving up.</li>
 *   <li><b>Latency correction.</b> A webcam frame is 30-100 ms old by the time we process it. At
 *       60 in/s that is up to 6 inches of stale position. We pair the frame with the pose from
 *       {@code frameAcquisitionNanoTime}, not the pose now. See {@link Odometry#getPoseAt(long)}.</li>
 * </ol>
 */
public class HiveTracker {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // Filtered field-frame estimate of the aim point.
    private double targetFieldX, targetFieldY, targetFieldZ;
    private boolean hasEstimate;
    private long lastSightingNano;

    // Most recent raw sighting, for telemetry.
    private double lastRange, lastBearingDeg, lastElevationDeg;
    private int lastPercentFound;
    private String lastClusterName = "none";
    private int detectionCount;

    public HiveTracker(HardwareMap hardwareMap) {
        // The default tag library is already getCurrentGameTagLibrary(), which in SDK 12.0 returns
        // the BIOBUZZ tags and their cluster definitions. We do not need to supply it.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Decimation 2 keeps a useful frame rate while still seeing the cluster across the field.
        // Lower it to 1 if you need more range; raise it to 3 if the loop time suffers.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConstants.Names.WEBCAM))
                .setCameraResolution(new Size(RobotConstants.Camera.WIDTH, RobotConstants.Camera.HEIGHT))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();
    }

    /**
     * Lock the exposure short and crank the gain.
     *
     * <p>This is not optional on a robot that shoots while driving. On auto-exposure the camera
     * lengthens the exposure in typical gym lighting, and a moving robot then smears the tag into
     * an unreadable blur exactly when you need it. A short fixed exposure trades brightness (which
     * gain buys back) for sharpness.
     *
     * <p>Call it once, after the portal is streaming. Blocks briefly.
     */
    public void setManualExposure() {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            long deadline = System.currentTimeMillis() + 5000;
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                    && System.currentTimeMillis() < deadline) {
                try { Thread.sleep(20); } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposure.setExposure(RobotConstants.Camera.EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        if (gain != null) gain.setGain(RobotConstants.Camera.GAIN);
    }

    /**
     * Process the newest detections and refresh the field-frame target estimate.
     *
     * @param odometry        already updated this loop
     * @param turretAngleRad  current turret angle; only used if the camera rides on the turret
     */
    public void update(Odometry odometry, double turretAngleRad) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        detectionCount = detections.size();

        AprilTagClusterDetection best = null;

        for (AprilTagDetection detection : detections) {
            // BIOBUZZ CELLs report as clusters. A CELL returns ONE cluster detection when at least
            // one member tag is visible; the member tags do not also appear as singletons. So any
            // singleton we see is something else and is not our target.
            if (!(detection instanceof AprilTagClusterDetection)) continue;

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null || cluster.ftcPose == null) continue;
            if (cluster.percentClusterFound < RobotConstants.Hive.MIN_PERCENT_CLUSTER_FOUND) continue;

            String wanted = RobotConstants.Hive.TARGET_CLUSTER_SHORT_NAME;
            if (wanted != null && !wanted.equals(cluster.metadata.shortName)) continue;

            // Prefer the sighting built from the most member tags -- its pose is the most stable.
            if (best == null || cluster.percentClusterFound > best.percentClusterFound) {
                best = cluster;
            }
        }

        if (best == null) return;

        lastRange = best.ftcPose.range;
        lastBearingDeg = best.ftcPose.bearing;
        lastElevationDeg = best.ftcPose.elevation;
        lastPercentFound = best.percentClusterFound;
        lastClusterName = best.metadata.shortName;

        // --- Camera frame -> robot frame -----------------------------------------------------
        // Using range/bearing/elevation rather than raw x/y/z: these three are unambiguously
        // documented (range = line-of-sight distance, +bearing = target is to our LEFT,
        // +elevation = target is ABOVE us) so there is no axis-convention trap.
        double bearing = Math.toRadians(lastBearingDeg);
        // ftcPose.elevation is relative to the camera's optical axis; add the mount tilt.
        // Exact when the tag is centred horizontally (which the turret drives toward).
        double elevation = Math.toRadians(lastElevationDeg + RobotConstants.Camera.PITCH_DEG);

        double horizontalRange = lastRange * Math.cos(elevation);
        double heightAboveCamera = lastRange * Math.sin(elevation);

        double cameraYaw = Math.toRadians(RobotConstants.Camera.YAW_DEG);
        if (RobotConstants.Camera.MOUNTED_ON_TURRET) cameraYaw += turretAngleRad;

        double angleInRobotFrame = cameraYaw + bearing;
        // Lens position in the robot frame. On a turret, the lens orbits the pivot.
        double camX = RobotConstants.Camera.X_IN;
        double camY = RobotConstants.Camera.Y_IN;
        if (RobotConstants.Camera.MOUNTED_ON_TURRET) {
            double tc = Math.cos(turretAngleRad), ts = Math.sin(turretAngleRad);
            camX = RobotConstants.Camera.TURRET_PIVOT_X_IN
                + RobotConstants.Camera.X_IN * tc - RobotConstants.Camera.Y_IN * ts;
            camY = RobotConstants.Camera.TURRET_PIVOT_Y_IN
                + RobotConstants.Camera.X_IN * ts + RobotConstants.Camera.Y_IN * tc;
        }
        double robotX = camX + horizontalRange * Math.cos(angleInRobotFrame);
        double robotY = camY + horizontalRange * Math.sin(angleInRobotFrame);
        double robotZ = RobotConstants.Camera.Z_IN + heightAboveCamera;

        // --- Robot frame -> field frame, using the pose from when the shutter opened ----------
        Odometry.TimedPose then = odometry.getPoseAt(best.frameAcquisitionNanoTime);
        double cos = Math.cos(then.headingRad);
        double sin = Math.sin(then.headingRad);
        double fieldX = then.x + robotX * cos - robotY * sin;
        double fieldY = then.y + robotX * sin + robotY * cos;
        double fieldZ = robotZ + RobotConstants.Hive.AIM_Z_BIAS_IN;

        // --- Blend into the running estimate --------------------------------------------------
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

    /**
     * True if we have an estimate recent enough to shoot at. Inside this window the estimate is
     * carried by odometry even with no camera sighting at all.
     */
    public boolean hasTarget() {
        return hasEstimate && getEstimateAgeSec() <= RobotConstants.Hive.MAX_ESTIMATE_AGE_SEC;
    }

    /** Seconds since the last accepted sighting. */
    public double getEstimateAgeSec() {
        if (!hasEstimate) return Double.POSITIVE_INFINITY;
        return (System.nanoTime() - lastSightingNano) / 1e9;
    }

    /** True if the camera is seeing the target right now, as opposed to dead-reckoning it. */
    public boolean isSightingLive() { return getEstimateAgeSec() < 0.25; }

    public double getTargetFieldX() { return targetFieldX; }
    public double getTargetFieldY() { return targetFieldY; }
    public double getTargetFieldZ() { return targetFieldZ; }

    /**
     * Throw away the estimate. Do this when the HIVE tips, or at the start of TeleOp, so a stale
     * pre-tip position cannot drag the aim.
     */
    public void resetEstimate() { hasEstimate = false; }

    // Telemetry accessors.
    public double getLastRange() { return lastRange; }
    public double getLastBearingDeg() { return lastBearingDeg; }
    public double getLastElevationDeg() { return lastElevationDeg; }
    public int getLastPercentFound() { return lastPercentFound; }
    public String getLastClusterName() { return lastClusterName; }
    public int getDetectionCount() { return detectionCount; }
    public List<AprilTagDetection> getRawDetections() { return aprilTag.getDetections(); }

    public void close() { if (visionPortal != null) visionPortal.close(); }

    public VisionPortal getVisionPortal() { return visionPortal; }
}
