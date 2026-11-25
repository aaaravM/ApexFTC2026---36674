package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds the BIOBUZZ balls in a MONOCHROME image. Runs on the VisionPortal's vision thread.
 *
 * <h2>Why this is shaped differently from a colour detector</h2>
 * The team's camera is an Arducam B0332 (OV9281 mono global shutter). There is no hue and no
 * saturation, so the old {@code BallTracker}'s approach -- {@code ColorBlobLocatorProcessor} with a
 * {@code ColorRange} per ball type -- is not merely worse here, it is inapplicable.
 *
 * <p>With colour, "find the orange pixels" does most of the work. Without it, finding round blobs is
 * the easy part and deciding <b>which round blobs are actually balls</b> is the whole problem:
 * every compliant wheel, mecanum roller, bolt head and ceiling light is a candidate. So this
 * processor spends little effort on thresholding and a lot on geometric plausibility. The three
 * filters that do the real work are described in {@link BallGeometry}; all three are nearly free.
 *
 * <h2>Thresholding both polarities</h2>
 * A ball is not reliably brighter than the tiles, nor reliably darker -- it depends on venue
 * lighting, the ball's finish, and where it sits relative to the lights. A detector that assumes one
 * polarity works at home and fails at a competition. So both are computed and OR-ed. Adaptive
 * thresholding rather than a single global cut handles the brightness gradient a wide lens always
 * has across the floor.
 *
 * <h2>What this deliberately does NOT do: the turret</h2>
 * Range, the horizon test, the predicted radius, the type decision and the consistency check are all
 * <b>yaw-independent</b> -- see {@link BallGeometry#groundPlanePositionLensFrame} for why. So this
 * processor does every filter without knowing the turret angle at all, and reports positions in the
 * LENS frame along with {@code captureTimeNanos}. The consumer applies the turret angle and robot
 * pose <b>as they were at capture time</b>, which on a slewing turret is not what they are when the
 * result is read.
 *
 * <h2>Threading</h2>
 * {@link #processFrame} runs on the vision thread; OpModes read {@link #getSightings()} from the
 * main thread. Results are published as a single immutable list behind one volatile write, so a
 * reader always sees a complete, self-consistent frame rather than a half-updated one. All Mats are
 * allocated once in {@link #init} and reused -- allocating per frame on Android invites GC pauses in
 * the middle of a match.
 */
public class MonoBallProcessor implements VisionProcessor {

    // =============================================================================================
    // Tunables.
    //
    // These live here rather than in RobotConstants on purpose: RobotConstants is being edited
    // heavily on several branches at once, and adding a section to it right now would conflict with
    // everyone. They are plain public statics so a tuning OpMode can still change them live.
    // =============================================================================================

    /** Gaussian blur kernel before thresholding. Odd; 0 disables. */
    public static int BLUR_KSIZE = 5;

    /**
     * Adaptive-threshold neighbourhood, odd, pixels. 0 means AUTO (recommended).
     *
     * <p>This must be comfortably LARGER than a ball's diameter in pixels, or a ball becomes its own
     * local background and dissolves. The catch is that it therefore depends on the capture
     * resolution: at 640 wide a near POLLEN is about 47 px across, at 1280 wide it is about 94 px.
     * A block size tuned at 640 and then used at 1280 silently stops seeing close balls -- which
     * reads as "the detector got worse when I improved the resolution".
     *
     * <p>So the default is auto: {@link #init} sets the effective value from the streaming width.
     * Set this non-zero only to override deliberately.
     */
    public static int ADAPTIVE_BLOCK = 0;

    /** Resolved in init() from the streaming width when {@link #ADAPTIVE_BLOCK} is 0. */
    private volatile int effectiveBlock = 51;

    /** How far from the local mean counts as foreground. Raise to reject noise. */
    public static int ADAPTIVE_C = 7;

    /** Structuring element size for OPEN and CLOSE. */
    public static int MORPH_KSIZE = 5;

    public static double MIN_RADIUS_PX = 4;
    public static double MAX_RADIUS_PX = 120;

    /** contourArea / (pi r^2). A disc is 1.0, a shaded ball 0.6-0.9, a strip of tape about 0.2. */
    public static double MIN_CIRCULARITY = 0.55;

    /** contourArea / boundingRect area. A cheap second shape test. */
    public static double MIN_FILL = 0.50;

    /**
     * Fraction subtracted from the measured radius to undo contact-shadow and morphology inflation.
     *
     * <p><b>This is a PLACEHOLDER from synthetic renders, not a measurement.</b> A ball resting on
     * the tiles casts a shadow; the dark-polarity threshold claims it and closing welds it on, so
     * the blob reads bigger than the ball. Uncorrected this measured about 45% large, which both
     * misclassified every POLLEN as NECTAR and rejected real balls as "matching no ball at this
     * range".
     *
     * <p>To measure it properly: put a ball at a tape-measured distance, read the radius this
     * processor reports, compare against {@link BallGeometry#predictedRadiusPx} for the true range,
     * and set this to the difference. It depends on real lighting, the real floor and the real ball
     * finish, so it must come from footage.
     */
    public static double SHADOW_BIAS_FRAC = 0.16;

    /** How far the measured radius may sit from a type's predicted radius, as a fraction. */
    public static double RADIUS_TOL_FRAC = 0.22;

    /** Beyond this, ignore detections. Rows just below the horizon give absurd ranges. */
    public static double MAX_RANGE_IN = 160.0;

    /** Reject when the two independent range methods disagree by more than this factor. */
    public static double CONSISTENCY_MAX = 1.35;

    /** Above this ratio, call it airborne rather than rejecting it. */
    public static double AIRBORNE_MIN = 1.35;

    /** Ball diameters considered, and the type each maps to. POLLEN first. */
    private static final BallType[] TYPES = { BallType.POLLEN, BallType.NECTAR_RED };
    private static final double[] DIAMETERS = { BallType.POLLEN.diameterIn,
                                                BallType.NECTAR_RED.diameterIn };

    // =============================================================================================
    // Result type
    // =============================================================================================

    /** One accepted ball, positioned in the LENS frame. Immutable. */
    public static final class Sighting {
        /** Blob centre in the image, pixels. */
        public final double u, v;
        /** Measured radius after shadow-bias correction, pixels. */
        public final double radiusPx;
        /**
         * Best guess at the ball type from apparent size alone. NECTAR_RED is reported for any
         * NECTAR: monochrome cannot distinguish red from blue, and pretending otherwise would be a
         * lie. Use the intake colour sensor for the alliance colour.
         */
        public final BallType type;
        /** True when the two types are far enough apart in apparent size here to tell them apart. */
        public final boolean typeTrustworthy;
        /** Position in the LENS frame: +X along the optical axis, +Y to its left, inches. */
        public final double xLensIn, yLensIn;
        /** Horizontal distance from the lens, inches. Yaw-independent, so valid as reported. */
        public final double rangeIn;
        /** Bearing from the optical axis, radians, CCW positive. Add the turret angle for robot frame. */
        public final double bearingFromAxisRad;
        /** Radius a real ball of this type would subtend at {@link #rangeIn}. */
        public final double predictedRadiusPx;
        public final double circularity;
        /** apparent-size range / ground-plane range. 1.0 means they agree. */
        public final double consistency;
        /** True when the ball appears further away than the floor model allows: it is in the air. */
        public final boolean airborne;
        /** Capture time of the frame this came from, from the VisionPortal. */
        public final long captureTimeNanos;

        Sighting(double u, double v, double radiusPx, BallType type, boolean typeTrustworthy,
                 double xLensIn, double yLensIn, double rangeIn, double predictedRadiusPx,
                 double circularity, double consistency, boolean airborne, long captureTimeNanos) {
            this.u = u; this.v = v; this.radiusPx = radiusPx;
            this.type = type; this.typeTrustworthy = typeTrustworthy;
            this.xLensIn = xLensIn; this.yLensIn = yLensIn; this.rangeIn = rangeIn;
            this.bearingFromAxisRad = Math.atan2(yLensIn, xLensIn);
            this.predictedRadiusPx = predictedRadiusPx;
            this.circularity = circularity;
            this.consistency = consistency;
            this.airborne = airborne;
            this.captureTimeNanos = captureTimeNanos;
        }

        /** Convert to the robot frame using the camera pose at this frame's capture time. */
        public BallGeometry.Position toRobotFrame(BallGeometry.CameraPose poseAtCapture) {
            return BallGeometry.lensFrameToRobot(xLensIn, yLensIn, poseAtCapture);
        }

        @Override public String toString() {
            return String.format("%s%s %.0fin %+.0fdeg r=%.1fpx%s",
                    type == BallType.POLLEN ? "POLLEN" : "NECTAR",
                    typeTrustworthy ? "" : "?",
                    rangeIn, Math.toDegrees(bearingFromAxisRad), radiusPx,
                    airborne ? " AIRBORNE" : "");
        }
    }

    // =============================================================================================
    // State
    // =============================================================================================

    private volatile List<Sighting> sightings = Collections.emptyList();
    private volatile int lastBlobCount = 0;
    private volatile int lastRejectCount = 0;
    private volatile String lastRejectReason = "";
    private volatile double lastProcessMs = 0;

    /** Camera intrinsics in use, at the streaming resolution. Null until init(). */
    private volatile BallGeometry.Intrinsics intrinsics;
    /** True when the SDK had no real calibration and a fallback is in use. */
    private volatile boolean calibrationIsFallback = true;
    private volatile String calibrationSource = "not initialised";

    private final double cameraHeightIn;
    private final double cameraPitchRad;

    // Reused buffers. Allocated once; never per frame.
    private Mat gray, blurred, bright, dark, mask, morphKernel, hierarchy;

    private final Paint ballPaint = new Paint();
    private final Paint rejectPaint = new Paint();
    private final Paint textPaint = new Paint();
    private final Paint horizonPaint = new Paint();

    /**
     * Uses {@link RobotConstants.Camera}, converting its pitch convention. See
     * {@link #PITCH_SIGN_WARNING}.
     */
    public MonoBallProcessor() {
        this(RobotConstants.Camera.Z_IN, -RobotConstants.Camera.PITCH_DEG);
    }

    /**
     * TWO OPPOSITE PITCH CONVENTIONS EXIST IN THIS CODEBASE. Read this before touching pitch.
     *
     * <ul>
     *   <li>{@code RobotConstants.Camera.PITCH_DEG} is <b>UP-positive</b>: its javadoc says
     *       "Degrees the camera is tilted UP from level", and {@code HiveTracker} line 174 uses it
     *       that way, adding it to an {@code ftcPose.elevation} where +elevation means the target is
     *       ABOVE us. That is self-consistent and is not being changed here.</li>
     *   <li>{@link BallGeometry} and this class are <b>DOWN-positive</b>, because a floor-looking
     *       camera is the normal case for ball detection and a down-positive angle keeps the
     *       ground-plane algebra free of negative signs.</li>
     * </ul>
     *
     * So every conversion between them must <b>negate</b>, and that is done explicitly at each
     * boundary rather than being left to whoever reads the code next.
     *
     * <p>Why this is worth a block comment: a pitch sign error is not a scale error. A wrong fx
     * makes every range wrong by one constant factor, which is correctable afterwards by editing a
     * number. A wrong pitch sign corrupts the ray DIRECTION, cannot be undone after the fact, and
     * throws nothing -- it just reports balls at plausible-looking wrong distances. At a 14 in lens,
     * a 10 degree sign flip is a 20 degree error, which is well over 100 in of range error at 72 in.
     *
     * <p>The ten-second check, which needs no checkerboard: {@link #getHorizonRow()} is drawn on the
     * Driver Station stream by this processor. Look at whether that line lands on the real
     * floor-to-wall junction. If it does not, the pitch sign or magnitude is wrong. That single
     * overlay validates cy, the pitch, and the whole camera-to-world rotation at once -- which is
     * something calibration cannot do, because its four numbers cannot be checked by eye.
     */
    public static final String PITCH_SIGN_WARNING =
            "RobotConstants.Camera.PITCH_DEG is UP-positive; this class is DOWN-positive. Negate.";

    /**
     * @param cameraHeightIn lens centre above the tiles, inches. The single most important mount
     *                       number: range accuracy scales directly with it (see
     *                       {@link BallGeometry}), so mounting higher is a free accuracy win.
     * @param pitchDownDeg   <b>DOWN-positive</b>, opposite to {@code RobotConstants.Camera.PITCH_DEG}.
     *                       See {@link #PITCH_SIGN_WARNING}.
     */
    public MonoBallProcessor(double cameraHeightIn, double pitchDownDeg) {
        this.cameraHeightIn = cameraHeightIn;
        this.cameraPitchRad = Math.toRadians(pitchDownDeg);

        ballPaint.setColor(Color.GREEN);
        ballPaint.setStyle(Paint.Style.STROKE);
        ballPaint.setStrokeWidth(3);
        rejectPaint.setColor(Color.argb(170, 200, 60, 60));
        rejectPaint.setStyle(Paint.Style.STROKE);
        rejectPaint.setStrokeWidth(2);
        horizonPaint.setColor(Color.argb(140, 255, 220, 0));
        horizonPaint.setStrokeWidth(2);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(22);
    }

    // =============================================================================================
    // VisionProcessor
    // =============================================================================================

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        gray = new Mat();
        blurred = new Mat();
        bright = new Mat();
        dark = new Mat();
        mask = new Mat();
        hierarchy = new Mat();
        morphKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(MORPH_KSIZE, MORPH_KSIZE));

        // Scale the adaptive-threshold neighbourhood to the resolution: roughly width/10, forced
        // odd. 640 -> 65, 800 -> 81, 1280 -> 129. Comfortably bigger than a near ball's diameter at
        // each, which is the condition that matters.
        effectiveBlock = ADAPTIVE_BLOCK > 0 ? (ADAPTIVE_BLOCK | 1) : Math.max(15, (width / 10) | 1);

        // PREFER THE SDK'S OWN CALIBRATION. If a teamwebcamcalibrations.xml entry matched this
        // camera's USB vid/pid, the SDK hands us real intrinsics for the ACTUAL streaming
        // resolution, which is strictly better than anything hard-coded -- it cannot get out of
        // sync with the resolution the way a constant can.
        //
        // The failure this guards against: when there is no matching entry, the SDK supplies a
        // FAKE calibration rather than throwing. Everything then runs and every distance is
        // silently wrong. isFake() is the only way to know, so it is checked and surfaced in
        // telemetry rather than swallowed.
        boolean usable = calibration != null
                && !calibration.isFake()
                && !calibration.isDegenerate()
                && calibration.focalLengthX > 1
                && calibration.focalLengthY > 1;

        if (usable) {
            intrinsics = new BallGeometry.Intrinsics(
                    calibration.focalLengthX, calibration.focalLengthY,
                    calibration.principalPointX, calibration.principalPointY,
                    width, height);
            calibrationIsFallback = false;
            calibrationSource = String.format("SDK calibration, fx=%.1f at %dx%d",
                    calibration.focalLengthX, width, height);
        } else {
            // Fall back to RobotConstants, scaled from its reference resolution to this one.
            intrinsics = new BallGeometry.Intrinsics(
                    RobotConstants.Camera.FX, RobotConstants.Camera.FY,
                    RobotConstants.Camera.CX, RobotConstants.Camera.CY,
                    RobotConstants.Camera.CAL_WIDTH, RobotConstants.Camera.CAL_HEIGHT)
                    .scaledTo(width, height);
            calibrationIsFallback = true;
            calibrationSource = String.format(
                    "NO SDK CALIBRATION (%s) -- using RobotConstants fx=%.1f scaled to %dx%d",
                    calibration == null ? "null" : calibration.isFake() ? "fake" : "degenerate",
                    intrinsics.fx, width, height);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long t0 = System.nanoTime();
        BallGeometry.Intrinsics intr = intrinsics;
        if (intr == null || frame == null || frame.empty()) return null;

        // The portal hands us RGB(A) even from a mono sensor, so collapse to one channel. Checking
        // the channel count rather than assuming: a 1-channel source would make cvtColor throw.
        if (frame.channels() >= 3) {
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);
        } else {
            frame.copyTo(gray);
        }

        int blur = BLUR_KSIZE | 1;
        if (blur >= 3) {
            Imgproc.GaussianBlur(gray, blurred, new org.opencv.core.Size(blur, blur), 0);
        } else {
            gray.copyTo(blurred);
        }

        int block = ADAPTIVE_BLOCK > 0 ? Math.max(3, ADAPTIVE_BLOCK | 1) : effectiveBlock;
        int c = Math.abs(ADAPTIVE_C);
        Imgproc.adaptiveThreshold(blurred, bright, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                Imgproc.THRESH_BINARY, block, -c);
        Imgproc.adaptiveThreshold(blurred, dark, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                Imgproc.THRESH_BINARY_INV, block, c);
        Core.bitwise_or(bright, dark, mask);

        // OPEN drops speckle. CLOSE heals the dark limb and the specular highlight that otherwise
        // split one ball into fragments and destroy its circularity. ONE closing pass, not two --
        // closing is what welds the contact shadow on, so each extra pass costs radius accuracy.
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, morphKernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, morphKernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Sighting> found = new ArrayList<>();
        int blobs = 0, rejects = 0;
        String reason = "";

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area <= 0) { contour.release(); continue; }

            org.opencv.core.MatOfPoint2f c2f = new org.opencv.core.MatOfPoint2f(contour.toArray());
            Point enclosingCentre = new Point();
            float[] enclosingRadius = new float[1];
            Imgproc.minEnclosingCircle(c2f, enclosingCentre, enclosingRadius);
            c2f.release();
            double er = enclosingRadius[0];
            if (er <= 0) { contour.release(); continue; }

            // Circularity uses the ENCLOSING circle: that is the shape test, and a ball welded to
            // its shadow legitimately is less circular.
            double circ = area / (Math.PI * er * er);
            if (circ < MIN_CIRCULARITY) { contour.release(); continue; }

            Rect box = Imgproc.boundingRect(contour);
            double fill = box.width * box.height > 0
                    ? area / (double) (box.width * box.height) : 0;
            if (fill < MIN_FILL) { contour.release(); continue; }

            // EQUIVALENT-AREA radius, not the enclosing circle. The enclosing circle is set by the
            // single furthest point, so the shadow crescent drags it out directly; area grows far
            // less because the crescent is thin. It also degrades gracefully for a ball clipped at
            // the frame edge, where an enclosing circle goes badly wrong.
            double radius = Math.sqrt(area / Math.PI) * (1.0 - SHADOW_BIAS_FRAC);
            if (radius < MIN_RADIUS_PX || radius > MAX_RADIUS_PX) { contour.release(); continue; }

            // CENTROID from moments, not the enclosing-circle centre: the shadow sits below the
            // ball and pulls that centre downward, and a lower image row reads as nearer.
            Moments m = Imgproc.moments(contour);
            double u = m.m00 > 0 ? m.m10 / m.m00 : enclosingCentre.x;
            double v = m.m00 > 0 ? m.m01 / m.m00 : enclosingCentre.y;
            contour.release();

            blobs++;
            Sighting s = classify(u, v, radius, circ, intr, captureTimeNanos);
            if (s != null) {
                found.add(s);
            } else {
                rejects++;
                reason = lastReason;
            }
        }
        contours.clear();

        found.sort((a, b) -> Double.compare(a.rangeIn, b.rangeIn));

        // One volatile write per frame, of an immutable list: a reader never sees a torn update.
        sightings = Collections.unmodifiableList(found);
        lastBlobCount = blobs;
        lastRejectCount = rejects;
        lastRejectReason = reason;
        lastProcessMs = (System.nanoTime() - t0) / 1e6;
        return null;
    }

    /** Set by {@link #classify} when it returns null, for telemetry. Vision thread only. */
    private String lastReason = "";

    /**
     * The three geometric filters, all yaw-independent so none of them needs the turret angle.
     *
     * @return a Sighting, or null with {@link #lastReason} set to which filter rejected it
     */
    private Sighting classify(double u, double v, double radiusPx, double circ,
                              BallGeometry.Intrinsics intr, long captureTimeNanos) {
        int bestIdx = -1;
        double bestAbsErr = Double.MAX_VALUE;
        double[] preds = new double[DIAMETERS.length];
        BallGeometry.Position[] pos = new BallGeometry.Position[DIAMETERS.length];

        for (int i = 0; i < DIAMETERS.length; i++) {
            BallGeometry.Position p = BallGeometry.groundPlanePositionLensFrame(
                    u, v, intr, cameraPitchRad, cameraHeightIn, DIAMETERS[i]);
            // FILTER 1: above the horizon, so it cannot be a ball resting on the floor.
            if (p == null) { lastReason = "above horizon"; return null; }
            if (p.rangeIn <= 0 || p.rangeIn > MAX_RANGE_IN) {
                lastReason = String.format("range %.0fin > max %.0f", p.rangeIn, MAX_RANGE_IN);
                return null;
            }
            double pred = predictedRadiusForRange(p.rangeIn, intr, DIAMETERS[i]);
            preds[i] = pred;
            pos[i] = p;
            double absErr = Math.abs(pred - radiusPx);
            if (absErr < bestAbsErr) { bestAbsErr = absErr; bestIdx = i; }
        }
        if (bestIdx < 0) { lastReason = "no candidate type"; return null; }

        double pred = preds[bestIdx];
        BallGeometry.Position p = pos[bestIdx];
        double relErr = Math.abs(pred - radiusPx) / pred;

        // FILTER 2: the radius must be plausible for SOME real ball at the range the floor implies.
        // This is the strong one -- it is what recovers most of the discrimination colour gave us.
        if (relErr > RADIUS_TOL_FRAC) {
            lastReason = String.format("r=%.1fpx matches no ball at %.0fin (pred %.1f, %.0f%% off)",
                    radiusPx, p.rangeIn, pred, 100 * relErr);
            return null;
        }

        double separation = Double.POSITIVE_INFINITY;
        for (int i = 0; i < preds.length; i++) {
            if (i != bestIdx) separation = Math.min(separation, Math.abs(pred - preds[i]));
        }

        // FILTER 3: do the two independent range methods agree?
        double slant = BallGeometry.apparentSizeSlantDistance(radiusPx, intr, DIAMETERS[bestIdx]);
        double dz = cameraHeightIn - DIAMETERS[bestIdx] / 2.0;
        double szRange = (!Double.isNaN(slant) && slant > Math.abs(dz))
                ? Math.sqrt(slant * slant - dz * dz) : Double.NaN;
        double consistency = Double.isNaN(szRange) ? Double.NaN : szRange / p.rangeIn;
        boolean airborne = false;
        if (!Double.isNaN(consistency)) {
            if (consistency >= AIRBORNE_MIN) {
                airborne = true;            // further than the floor allows: not on the floor
            } else if (consistency > CONSISTENCY_MAX || consistency < 1.0 / CONSISTENCY_MAX) {
                lastReason = String.format("range methods disagree (floor %.0f, size %.0f)",
                        p.rangeIn, szRange);
                return null;
            }
        }

        return new Sighting(u, v, radiusPx, TYPES[bestIdx],
                separation >= BallGeometry.MIN_TYPE_SEPARATION_PX,
                p.xIn, p.yIn, p.rangeIn, pred, circ, consistency, airborne, captureTimeNanos);
    }

    /** Predicted radius without needing a CameraPose, since this class holds height directly. */
    private double predictedRadiusForRange(double rangeIn, BallGeometry.Intrinsics intr,
                                           double diameterIn) {
        double dz = cameraHeightIn - diameterIn / 2.0;
        double L = Math.hypot(rangeIn, dz);
        return L <= 0 ? Double.NaN : intr.fx * (diameterIn / 2.0) / L;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        BallGeometry.Intrinsics intr = intrinsics;
        if (canvas == null || intr == null) return;

        // The horizon, drawn because it explains most "why can it not see that ball" questions at
        // a glance: anything above this line is rejected by construction.
        float hy = (float) ((intr.cy - intr.fy * Math.tan(cameraPitchRad)) * scaleBmpPxToCanvasPx);
        canvas.drawLine(0, hy, onscreenWidth, hy, horizonPaint);

        List<Sighting> snapshot = sightings;
        for (Sighting s : snapshot) {
            float cx = (float) (s.u * scaleBmpPxToCanvasPx);
            float cy = (float) (s.v * scaleBmpPxToCanvasPx);
            float r = (float) (s.radiusPx * scaleBmpPxToCanvasPx);
            ballPaint.setColor(s.airborne ? Color.CYAN
                    : s.typeTrustworthy ? Color.GREEN : Color.YELLOW);
            canvas.drawCircle(cx, cy, r, ballPaint);
            canvas.drawText(s.toString(), cx - r, cy - r - 6, textPaint);
        }
        if (calibrationIsFallback) {
            textPaint.setColor(Color.RED);
            canvas.drawText("UNCALIBRATED - ranges unreliable", 10, onscreenHeight - 14, textPaint);
            textPaint.setColor(Color.WHITE);
        }
    }

    // =============================================================================================
    // Accessors, all safe from the OpMode thread
    // =============================================================================================

    /** Newest frame's accepted balls, nearest first. Never null; immutable. */
    public List<Sighting> getSightings() { return sightings; }

    /** Nearest accepted ball, or null. */
    public Sighting getClosest() {
        List<Sighting> s = sightings;
        return s.isEmpty() ? null : s.get(0);
    }

    /** Round blobs that passed the shape tests last frame, before the geometric filters. */
    public int getBlobCount() { return lastBlobCount; }

    /** How many of those the geometric filters threw away last frame. */
    public int getRejectCount() { return lastRejectCount; }

    /** Why the last rejected blob was rejected. The first thing to read when nothing is detected. */
    public String getLastRejectReason() { return lastRejectReason; }

    /** Milliseconds the last frame took in this processor. */
    public double getProcessMs() { return lastProcessMs; }

    /** Intrinsics actually in use, at the streaming resolution. Null before init. */
    public BallGeometry.Intrinsics getIntrinsics() { return intrinsics; }

    /**
     * True when the SDK supplied no real calibration, so hard-coded numbers are being scaled
     * instead. <b>Surface this in telemetry.</b> Every reported distance is only as good as fx.
     */
    public boolean isCalibrationFallback() { return calibrationIsFallback; }

    /** Human-readable account of where the intrinsics came from. */
    public String getCalibrationSource() { return calibrationSource; }

    /** The adaptive-threshold block size actually in use, after the auto-scaling in init(). */
    public int getEffectiveBlock() { return effectiveBlock; }

    /**
     * What fx SHOULD be, from the lens's published specification, as a check on calibration.
     *
     * <p>Arducam's B0332 datasheet gives the M12 lens as <b>70 degrees horizontal FOV, EFL
     * 2.8 mm</b>, on a 1/4 inch 1280x800 sensor with 3 um pixels. So
     * {@code fx = (width/2) / tan(HFOV/2)}, which at 1280 wide is
     * {@code 640 / tan(35 deg) = 914}. At 640 wide it is 457.
     *
     * <p>Compare a fresh calibration against this. A reprojection error can look respectable while
     * the focal length is badly wrong (too few distinct board poses does exactly that), and this
     * physical cross-check catches what the reprojection error will not. Agreement within a few
     * percent is expected; a 20% disagreement means the calibration, not the lens.
     *
     * <p>Caveat: this holds for the NATIVE 16:10 modes. A 4:3 mode such as 640x480 may crop rather
     * than scale the sensor, which changes the field of view and moves the principal point, so the
     * simple ratio does not carry across aspect ratios.
     */
    public static double expectedFxForWidth(int width) {
        final double hfovDeg = 70.0;
        return (width / 2.0) / Math.tan(Math.toRadians(hfovDeg / 2.0));
    }

    /** Image row of the horizon, for overlays and sanity checks. */
    public double getHorizonRow() {
        BallGeometry.Intrinsics intr = intrinsics;
        return intr == null ? Double.NaN : intr.cy - intr.fy * Math.tan(cameraPitchRad);
    }
}
