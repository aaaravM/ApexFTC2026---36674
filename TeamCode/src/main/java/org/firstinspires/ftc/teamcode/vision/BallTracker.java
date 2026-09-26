package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Finds POLLEN and NECTAR on the floor and reports where they are relative to the robot.
 *
 * <h2>About the trained-model plan -- please read</h2>
 * You mentioned training a CV model to track the balls. Two things changed that you need to know:
 *
 * <p><b>1. TensorFlow Object Detection is gone from the FTC SDK.</b> The release notes for SDK 10.0
 * say plainly: <i>"Java classes and Blocks for TensorFlow Object Detection have been removed."</i>
 * There is no {@code TfodProcessor} in SDK 12.0, and FIRST's old training toolchain (fmltc) is an
 * archived repository. So the built-in path you may have read about in older tutorials no longer
 * exists.
 *
 * <p><b>2. For this specific task, a neural net is probably the worse tool anyway.</b> A CNN earns
 * its keep when the target varies -- different shapes, poses, lighting, occlusion. Here the targets
 * are uniformly coloured spheres in three known colours on a known field. A colour threshold plus a
 * circularity filter gets you position and range at 30+ fps using a few percent of the CPU, and it
 * is debuggable: when it misbehaves you can see exactly which pixels matched. A CNN on the Control
 * Hub's CPU would run at a handful of fps, cost you loop time you need for the turret, and fail in
 * ways you cannot inspect from the driver's station. That is what this class does instead, using the
 * SDK's {@link ColorBlobLocatorProcessor} with its {@code BY_CIRCULARITY} filter.
 *
 * <p><b>If you still want a real trained detector</b>, the supported route is a <b>Limelight 3A</b>.
 * It is FTC-legal, it has its own processor so it costs the Control Hub nothing, the SDK has a
 * {@code Limelight3A} class and a {@code SensorLimelight3A} sample, and SDK 12.0 can even upload
 * pipelines from code so you can version-control them. You train on Roboflow and flash the pipeline.
 * That is the plan to pick if you want the ML experience -- not on-Hub inference.
 *
 * <h2>Camera</h2>
 * This uses a second camera aimed down at the floor ({@link RobotConstants.Names#WEBCAM_BALLS}),
 * because the tag camera has to look up at the HIVE at the same time. If you only have one camera
 * for now, delete this class's portal and instead add the blob processors to
 * {@link HiveTracker}'s {@code VisionPortal.Builder} -- one portal can hold several processors, and
 * you toggle them with {@code visionPortal.setProcessorEnabled(processor, boolean)}. Expect to lose
 * frame rate when more than one is enabled.
 */
public class BallTracker {

    /** One ball sighting, in the robot frame. */
    public static final class Sighting {
        /** Which kind of ball, inferred from which colour processor found it. */
        public final BallType type;
        /** Straight-line distance from the camera, inches, from the apparent size. */
        public final double distanceIn;
        /** Angle to the ball, radians, positive to our LEFT (same sign rule as everywhere else). */
        public final double bearingRad;
        /** Forward distance in the robot frame, inches. */
        public final double robotX;
        /** Leftward distance in the robot frame, inches. */
        public final double robotY;
        /** Blob radius in pixels. Small radius means a long, unreliable estimate. */
        public final double radiusPx;
        /** 0-1, how circular the blob was. Under ~0.7 it is probably two balls touching. */
        public final double circularity;

        Sighting(BallType type, double distanceIn, double bearingRad,
                 double robotX, double robotY, double radiusPx, double circularity) {
            this.type = type;
            this.distanceIn = distanceIn;
            this.bearingRad = bearingRad;
            this.robotX = robotX;
            this.robotY = robotY;
            this.radiusPx = radiusPx;
            this.circularity = circularity;
        }

        public double getBearingDeg() { return Math.toDegrees(bearingRad); }

        @Override public String toString() {
            return String.format("%s  %.1f in @ %+.0f deg  (r=%.0fpx circ=%.2f)",
                    type, distanceIn, getBearingDeg(), radiusPx, circularity);
        }
    }

    /** Minimum blob area in pixels. Below this it is noise. */
    private static final double MIN_AREA_PX = 80;
    private static final double MAX_AREA_PX = 40000;
    /** Below this circularity it is not a single ball. Shadows push this down; tune on the field. */
    private static final double MIN_CIRCULARITY = 0.65;

    private final Map<BallType, ColorBlobLocatorProcessor> processors = new EnumMap<>(BallType.class);
    private final VisionPortal visionPortal;
    private List<Sighting> sightings = Collections.emptyList();

    /**
     * @param hardwareMap standard
     * @param lookFor     which ball types to search for. Each one costs CPU, so ask for only what
     *                    you need -- usually POLLEN plus your own alliance's NECTAR.
     */
    public BallTracker(HardwareMap hardwareMap, BallType... lookFor) {
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConstants.Names.WEBCAM_BALLS))
                // 320x240 is deliberate. Colour blobs do not get better with resolution, and the
                // lower resolution roughly quarters the per-frame cost.
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true);

        for (BallType type : lookFor) {
            ColorRange range = colorRangeFor(type);
            if (range == null) continue;
            ColorBlobLocatorProcessor processor = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(range)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBoxFitColor(0)                          // no rectangles; balls are round
                    .setCircleFitColor(0xFFFFFF00)              // yellow circle overlay
                    .setBlurSize(5)
                    // CLOSING fills the specular highlight that a shiny ball puts in the middle of
                    // the blob, which would otherwise split one ball into a ring of fragments.
                    .setDilateSize(8)
                    .setErodeSize(8)
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                    .build();
            processors.put(type, processor);
            builder.addProcessor(processor);
        }

        visionPortal = builder.build();
    }

    private static ColorRange colorRangeFor(BallType type) {
        switch (type) {
            case POLLEN:      return ColorRange.YELLOW;
            case NECTAR_RED:  return ColorRange.RED;
            case NECTAR_BLUE: return ColorRange.BLUE;
            default:          return null;
        }
    }

    /** Re-read every enabled processor and rebuild the sighting list. Call once per loop. */
    public void update() {
        List<Sighting> found = new ArrayList<>();

        for (Map.Entry<BallType, ColorBlobLocatorProcessor> entry : processors.entrySet()) {
            BallType type = entry.getKey();
            List<ColorBlobLocatorProcessor.Blob> blobs = entry.getValue().getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    MIN_AREA_PX, MAX_AREA_PX, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    MIN_CIRCULARITY, 1.0, blobs);

            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                Circle circle = blob.getCircle();
                if (circle == null || circle.getRadius() < 3) continue;
                found.add(toSighting(type, circle, blob.getCircularity()));
            }
        }

        // Nearest first -- that is almost always the one you want to drive to.
        found.sort((a, b) -> Double.compare(a.distanceIn, b.distanceIn));
        sightings = found;
    }

    /**
     * Pinhole projection, run backwards.
     *
     * <p>A sphere of known real diameter {@code D} at distance {@code z} projects to a pixel
     * diameter of {@code fx * D / z}. We measured the pixel diameter ({@code 2 * radius}), and we
     * know {@code D} because the colour told us which ball it is -- POLLEN is 2.8 in and NECTAR is
     * 3.6 in. So {@code z = fx * D / (2 * radius)}.
     *
     * <p>Note how the colour classification is doing double duty: it picks the shooter RPM later,
     * and right here it is what makes the range estimate possible at all. Without knowing which
     * ball it is, apparent size is ambiguous -- a near POLLEN and a far NECTAR look identical.
     */
    private Sighting toSighting(BallType type, Circle circle, double circularity) {
        double radiusPx = circle.getRadius();
        double distanceIn = RobotConstants.Camera.FX * type.diameterIn / (2.0 * radiusPx);

        // Pixels right of the principal point are to our right, which is a NEGATIVE bearing.
        double bearingRad = Math.atan2(RobotConstants.Camera.CX - circle.getX(),
                                       RobotConstants.Camera.FX);

        double robotX = RobotConstants.Camera.X_IN + distanceIn * Math.cos(bearingRad);
        double robotY = RobotConstants.Camera.Y_IN + distanceIn * Math.sin(bearingRad);

        return new Sighting(type, distanceIn, bearingRad, robotX, robotY, radiusPx, circularity);
    }

    /** All balls seen this frame, nearest first. Never null. */
    public List<Sighting> getSightings() { return sightings; }

    /** The nearest ball of any tracked type, or null. */
    public Sighting getClosest() {
        return sightings.isEmpty() ? null : sightings.get(0);
    }

    /** The nearest ball our alliance can actually score, or null. */
    public Sighting getClosestScorable() {
        for (Sighting s : sightings) {
            if (s.type.isScorableBy(RobotConstants.alliance)) return s;
        }
        return null;
    }

    /** Turn a processor on or off to buy back loop time when you are not intaking. */
    public void setEnabled(BallType type, boolean enabled) {
        ColorBlobLocatorProcessor processor = processors.get(type);
        if (processor != null) visionPortal.setProcessorEnabled(processor, enabled);
    }

    public void close() { if (visionPortal != null) visionPortal.close(); }

    public VisionPortal getVisionPortal() { return visionPortal; }
}
