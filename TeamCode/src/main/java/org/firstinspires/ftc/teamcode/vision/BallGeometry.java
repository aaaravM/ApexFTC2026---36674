package org.firstinspires.ftc.teamcode.vision;

/**
 * Pixel-to-field geometry for monochrome ball detection.
 *
 * <h2>No Android imports, deliberately</h2>
 * Nothing in this file touches the Android SDK, the FTC SDK, or OpenCV. That is on purpose: it can
 * be compiled and <b>executed on a laptop</b> with plain {@code javac}, which is how it was checked
 * against the Python reference implementation in {@code tools/vision/ball_geometry.py} rather than
 * merely assumed to match. See {@code tools/vision/crosscheck_java.py}.
 *
 * <p>The same trick is used by {@link org.firstinspires.ftc.teamcode.pedro.FollowerTuning}, and for
 * the same reason: geometry and tuning numbers are exactly the things you want to test without a
 * robot in the loop.
 *
 * <h2>Range comes from the FLOOR, not from the ball's apparent size</h2>
 * There are two ways to get range from one camera.
 *
 * <ol>
 *   <li><b>Apparent size.</b> A ball of known diameter D subtending radius r pixels is at slant
 *       distance {@code L = fx * (D/2) / r}. This is what the old colour {@code BallTracker} did.</li>
 *   <li><b>The ground plane.</b> A ball resting on the tiles has its centre at a known height D/2.
 *       Back-project the pixel into a ray, intersect that ray with the plane {@code z = D/2}, and
 *       range falls out of the geometry.</li>
 * </ol>
 *
 * Method 2 wins here, for three separate reasons:
 *
 * <ul>
 *   <li><b>Accuracy.</b> Differentiating both models, the range error per pixel of measurement
 *       error is {@code L^2/(fy*H)} for the ground plane against {@code L^2/(fx*D/2)} for apparent
 *       size, where H is the camera height above the ball centre. The ratio is {@code H/(D/2)} --
 *       about 9x for a camera 14 in up and a 2.8 in POLLEN, and it does <b>not</b> depend on range.
 *       A direct consequence: <b>mounting the camera higher buys range accuracy for free.</b></li>
 *   <li><b>Measurement noise.</b> A blob's centroid is an average over hundreds of pixels and is
 *       stable to a fraction of a pixel. Its radius depends on exactly where the threshold cuts the
 *       soft, shaded edge of the ball, which moves with lighting. Method 2 uses the better input.</li>
 *   <li><b>It breaks the circular dependency.</b> In monochrome we cannot tell POLLEN from NECTAR by
 *       colour, so we do not know D -- but the plane {@code z = D/2} barely depends on D (0.4 in of
 *       difference against a ~14 in camera height, about 3% of range). So: get range first without
 *       knowing the type, then use the measured radius to decide the type. See
 *       {@link #classifyByRadius}.</li>
 * </ul>
 *
 * Method 1 still earns its keep: it is the only option for a ball in <i>flight</i>, where the floor
 * assumption is false, and disagreement between the two is a free airborne-ball detector.
 *
 * <h2>The free false-positive filter</h2>
 * Without colour, anything round and mid-grey is a candidate -- other robots' compliant wheels and
 * mecanum rollers, bolt heads, ceiling lights. The ground-plane model hands us a strong filter for
 * nothing: <b>a ball on the floor must appear BELOW THE HORIZON.</b> A ray through a pixel above the
 * horizon rises forever and never meets the floor, so {@link #groundPlanePosition} returns null.
 * That one test discards everything on the field wall, on another robot's superstructure, and in the
 * lighting rig.
 *
 * <h2>Frame conventions, matching the rest of this codebase</h2>
 * Robot/field frame: <b>+X forward, +Y left, +Z up</b>, inches and radians, heading CCW positive.
 * Camera frame (OpenCV): +x right, <b>+y DOWN</b>, +z forward along the optical axis.
 * Camera pitch is positive when tilted <b>DOWN</b> toward the floor.
 */
public final class BallGeometry {

    private BallGeometry() {}

    /**
     * Below this gap between the two types' predicted radii, size cannot tell them apart.
     *
     * <p><b>4 px, not 1.5.</b> The radius measurement noise on a real blob is on the order of
     * sigma = 1 px -- it is set by where the threshold cuts the ball's soft shaded edge, plus the
     * contact-shadow bias. A 1.5 px threshold is therefore about 1.5 sigma, which means roughly a
     * one-in-four chance of being wrong at the boundary while still reporting the answer as
     * trustworthy. That is worse than useless: a confidently wrong ball type is acted on, an
     * admitted unknown falls back to the intake colour sensor.
     *
     * <p>At 4 px (4 sigma) the honest typing envelope is roughly 48-60 in at 640 wide and 72-96 in
     * at 1280 wide. Beyond that this reports {@code trustworthy() == false} and the caller should
     * use the colour sensor, which knows for certain once the ball is in hand.
     *
     * <p>Re-derive this once sigma is measured rather than assumed: roll a ball through the
     * detector, log radius against range, and take the residual spread.
     */
    public static final double MIN_TYPE_SEPARATION_PX = 4.0;

    // =============================================================================================
    // Value types
    // =============================================================================================

    /**
     * Pinhole intrinsics at a specific capture resolution.
     *
     * <p><b>Always carry the resolution with the numbers.</b> Focal length scales linearly with
     * capture width, so an fx measured at 1280 is exactly twice the fx that applies at 640. Using
     * the wrong one is a silent factor-of-two error in every range, with no exception thrown.
     * {@link #scaledTo} exists so that mistake is hard to make by accident.
     */
    public static final class Intrinsics {
        public final double fx, fy, cx, cy;
        public final int width, height;

        public Intrinsics(double fx, double fy, double cx, double cy, int width, int height) {
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
            this.width = width; this.height = height;
        }

        /**
         * Same lens, different capture size.
         *
         * <p><b>Only valid when the sensor SCALES</b> (bins or resamples the whole field of view).
         * If it CROPS instead -- which is what changing aspect ratio often does, e.g. 1280x800 at
         * 16:10 down to 640x480 at 4:3 -- the field of view changes and the principal point moves,
         * and this scaling is simply wrong. Calibrate at each aspect ratio you intend to use.
         */
        public Intrinsics scaledTo(int newWidth, int newHeight) {
            double sx = newWidth / (double) width;
            double sy = newHeight / (double) height;
            return new Intrinsics(fx * sx, fy * sy, cx * sx, cy * sy, newWidth, newHeight);
        }

        public double hfovDeg() { return 2.0 * Math.toDegrees(Math.atan(width / (2.0 * fx))); }
        public double vfovDeg() { return 2.0 * Math.toDegrees(Math.atan(height / (2.0 * fy))); }
    }

    /**
     * Where the lens is and where it points.
     *
     * <p>For a TURRET-MOUNTED camera, {@code xIn}/{@code yIn} are the lens offset <b>from the
     * turret pivot</b> measured with the turret at zero, and the lens <i>orbits</i> that pivot as
     * the turret rotates. {@link #lensXInRobotFrame} handles that. Forgetting it is a real error: a
     * lens 4 in ahead of the pivot moves 8 in between turret +90 and -90 degrees.
     */
    public static final class CameraPose {
        public final double heightIn;
        public final double pitchRad;    // positive = tilted DOWN
        public final double xIn, yIn;
        public final double yawRad;      // lens direction relative to robot forward (turret angle)
        public final double pivotXIn, pivotYIn;
        public final boolean onTurret;

        public CameraPose(double heightIn, double pitchDeg, double xIn, double yIn,
                          double yawRad, double pivotXIn, double pivotYIn, boolean onTurret) {
            this.heightIn = heightIn;
            this.pitchRad = Math.toRadians(pitchDeg);
            this.xIn = xIn; this.yIn = yIn;
            this.yawRad = yawRad;
            this.pivotXIn = pivotXIn; this.pivotYIn = pivotYIn;
            this.onTurret = onTurret;
        }

        /** Chassis-mounted, pointing straight ahead. */
        public static CameraPose chassis(double heightIn, double pitchDeg, double xIn, double yIn) {
            return new CameraPose(heightIn, pitchDeg, xIn, yIn, 0, 0, 0, false);
        }

        /** Same mount, new turret angle. Cheap enough to rebuild every loop. */
        public CameraPose withYaw(double newYawRad) {
            return new CameraPose(heightIn, Math.toDegrees(pitchRad), xIn, yIn,
                    newYawRad, pivotXIn, pivotYIn, onTurret);
        }

        public double lensXInRobotFrame() {
            if (!onTurret) return xIn;
            return pivotXIn + (xIn * Math.cos(yawRad) - yIn * Math.sin(yawRad));
        }

        public double lensYInRobotFrame() {
            if (!onTurret) return yIn;
            return pivotYIn + (xIn * Math.sin(yawRad) + yIn * Math.cos(yawRad));
        }
    }

    /** A ball's position, as recovered from one pixel. */
    public static final class Position {
        /** Robot frame, inches. +X forward, +Y left. */
        public final double xIn, yIn;
        /** Horizontal distance from the robot's tracking point, inches. */
        public final double rangeIn;

        public Position(double xIn, double yIn, double rangeIn) {
            this.xIn = xIn; this.yIn = yIn; this.rangeIn = rangeIn;
        }

        /** Bearing in the robot frame, radians, CCW positive. */
        public double bearingRad() { return Math.atan2(yIn, xIn); }
    }

    /** The outcome of deciding which ball type a blob is, using size alone. */
    public static final class TypeGuess {
        /** Index into the diameters array that was passed in. */
        public final int index;
        public final double diameterIn;
        public final Position position;
        /** What radius this type would subtend at {@link Position#rangeIn}. */
        public final double predictedRadiusPx;
        /** How far the measurement sits from that, as a fraction of it. */
        public final double radiusErrorFrac;
        /**
         * Gap between the two best candidates' predicted radii, in pixels. When this falls below
         * {@link #MIN_TYPE_SEPARATION_PX} the types are physically indistinguishable at this range
         * no matter how good the detector is, and the intake colour sensor is the only real source.
         */
        public final double separationPx;

        public TypeGuess(int index, double diameterIn, Position position,
                         double predictedRadiusPx, double radiusErrorFrac, double separationPx) {
            this.index = index; this.diameterIn = diameterIn; this.position = position;
            this.predictedRadiusPx = predictedRadiusPx;
            this.radiusErrorFrac = radiusErrorFrac;
            this.separationPx = separationPx;
        }

        public boolean trustworthy() { return separationPx >= MIN_TYPE_SEPARATION_PX; }
    }

    // =============================================================================================
    // Core geometry
    // =============================================================================================

    /**
     * Rotate a camera-frame ray {@code (a, b, 1)} into the world frame, before yaw.
     *
     * <p>The camera basis expressed in world coordinates for a lens pitched DOWN by p:
     * <pre>
     *   x_cam (right)   -&gt; ( 0,      -1,  0      )
     *   y_cam (down)    -&gt; (-sin p,   0, -cos p  )
     *   z_cam (forward) -&gt; ( cos p,   0, -sin p  )
     * </pre>
     * Right-handed: {@code x_cam cross y_cam == z_cam}, which the cross-check asserts.
     *
     * @return {@code {rx, ry, rz}}; not normalised, which is all the callers need.
     */
    static double[] rayCameraToWorld(double a, double b, double pitchRad) {
        double sp = Math.sin(pitchRad), cp = Math.cos(pitchRad);
        return new double[] { cp - b * sp, -a, -sp - b * cp };
    }

    /**
     * Where a FLOOR-RESTING ball seen at pixel (u,v) is, in the robot frame. The primary method.
     *
     * @return the position, or <b>null</b> if the ray never meets the floor -- meaning the pixel is
     *         at or above the horizon, so this cannot be a ball on the ground. That null is a
     *         feature: use it as a false-positive filter before anything expensive.
     */
    public static Position groundPlanePosition(double u, double v, Intrinsics intr,
                                               CameraPose pose, double ballDiameterIn) {
        double a = (u - intr.cx) / intr.fx;
        double b = (v - intr.cy) / intr.fy;
        double[] r = rayCameraToWorld(a, b, pose.pitchRad);
        double rx = r[0], ry = r[1], rz = r[2];

        // Rotate by the lens yaw (turret angle) into the robot frame. Yaw is about Z, so rz is
        // unaffected -- which is why the horizon test below is yaw-independent.
        double cy = Math.cos(pose.yawRad), sy = Math.sin(pose.yawRad);
        double wx = rx * cy - ry * sy;
        double wy = rx * sy + ry * cy;

        double targetZ = ballDiameterIn / 2.0;
        double dz = targetZ - pose.heightIn;       // negative: ball centre is below the lens
        if (rz >= -1e-9) return null;              // ray level or rising: never reaches the floor
        double t = dz / rz;                        // both negative, so t is positive
        if (t <= 0) return null;

        double bx = pose.lensXInRobotFrame() + t * wx;
        double by = pose.lensYInRobotFrame() + t * wy;
        return new Position(bx, by, Math.hypot(bx, by));
    }

    /**
     * Same intersection, but in the LENS frame: yaw zero and no mount offset applied.
     *
     * <h3>Why this split exists, and why it is not just a convenience</h3>
     * The camera rides on the turret, so the yaw changes continuously. But look at what yaw actually
     * affects in {@link #groundPlanePosition}: yaw rotates about Z, so {@code rz} is untouched,
     * which means {@code t} is untouched, which means <b>RANGE is yaw-independent</b> -- and so are
     * the horizon test, the predicted radius, the type classification and the consistency check.
     * Only the final {@code (x, y)} direction needs the yaw.
     *
     * <p>That is what lets the vision thread do all the expensive filtering with no knowledge of
     * the turret at all, and lets the consumer apply {@link #lensFrameToRobot} afterwards using the
     * turret angle <b>recorded at the frame's capture time</b> rather than whatever it happens to be
     * when the result is read. On a slewing turret those are not the same angle.
     *
     * @return position in the lens frame (+X along the optical axis on the floor, +Y to its left),
     *         or null if the pixel is at or above the horizon
     */
    public static Position groundPlanePositionLensFrame(double u, double v, Intrinsics intr,
                                                        double pitchRad, double heightIn,
                                                        double ballDiameterIn) {
        double a = (u - intr.cx) / intr.fx;
        double b = (v - intr.cy) / intr.fy;
        double[] r = rayCameraToWorld(a, b, pitchRad);
        double rz = r[2];
        if (rz >= -1e-9) return null;
        double t = (ballDiameterIn / 2.0 - heightIn) / rz;
        if (t <= 0) return null;
        double x = t * r[0], y = t * r[1];
        return new Position(x, y, Math.hypot(x, y));
    }

    /**
     * Lens-frame point to robot frame: rotate by the lens yaw, then add the lens position.
     *
     * <p>Pass the pose carrying the turret angle <b>from the frame's capture time</b>. Composing
     * this with {@link #groundPlanePositionLensFrame} gives exactly {@link #groundPlanePosition},
     * which {@code CrossCheck} asserts rather than assumes.
     */
    public static Position lensFrameToRobot(double xLens, double yLens, CameraPose pose) {
        double c = Math.cos(pose.yawRad), s = Math.sin(pose.yawRad);
        double bx = pose.lensXInRobotFrame() + xLens * c - yLens * s;
        double by = pose.lensYInRobotFrame() + xLens * s + yLens * c;
        return new Position(bx, by, Math.hypot(bx, by));
    }

    /**
     * Slant (3-D) distance from the lens to the ball centre, from its apparent radius.
     *
     * <p>{@code L = fx * (D/2) / r}. This is the small-angle form; the exact relation for a sphere
     * is {@code r = f*R/sqrt(L^2 - R^2)}, which differs by 0.5% at L = 10R and less beyond, so the
     * simple form is always fine at useful ranges.
     *
     * @return the distance, or {@link Double#NaN} for a non-positive radius.
     */
    public static double apparentSizeSlantDistance(double radiusPx, Intrinsics intr,
                                                   double ballDiameterIn) {
        if (radiusPx <= 0) return Double.NaN;
        return intr.fx * (ballDiameterIn / 2.0) / radiusPx;
    }

    /**
     * Horizontal range from apparent size, for cross-checking the ground-plane answer.
     *
     * @return the range, or NaN when the slant distance is shorter than the height difference --
     *         geometrically impossible, so it indicates a bad radius measurement.
     */
    public static double apparentSizeGroundRange(double radiusPx, Intrinsics intr,
                                                 CameraPose pose, double ballDiameterIn) {
        double L = apparentSizeSlantDistance(radiusPx, intr, ballDiameterIn);
        if (Double.isNaN(L)) return Double.NaN;
        double dz = pose.heightIn - ballDiameterIn / 2.0;
        if (L <= Math.abs(dz)) return Double.NaN;
        return Math.sqrt(L * L - dz * dz);
    }

    /**
     * What radius a ball of this diameter WOULD subtend at this horizontal range.
     *
     * <p>Inverse of {@link #apparentSizeGroundRange}, and the heart of classification without
     * colour: get range from the floor, then ask which ball's predicted radius matches what was
     * measured.
     */
    public static double predictedRadiusPx(double rangeIn, Intrinsics intr,
                                           CameraPose pose, double ballDiameterIn) {
        double dz = pose.heightIn - ballDiameterIn / 2.0;
        double L = Math.hypot(rangeIn, dz);
        if (L <= 0) return Double.NaN;
        return intr.fx * (ballDiameterIn / 2.0) / L;
    }

    /**
     * Decide which ball type a blob is, from size alone, with no colour information.
     *
     * <p>Method: range from the ground plane (which barely depends on D, so it can be computed
     * before the type is known -- this is what breaks the circularity), then the radius each
     * candidate would subtend at that range, then pick the closest match.
     *
     * <h3>The match is on ABSOLUTE pixel difference, not relative</h3>
     * This matters, and it was a real bug caught by {@code tools/vision/crosscheck_java.py} when
     * this method sorted by {@code |pred - r| / pred} while the Python reference sorted by
     * {@code |pred - r|}. Two reasons absolute is right:
     * <ul>
     *   <li>Radius measurement noise is roughly <b>constant in pixels</b> -- it comes from where
     *       the threshold cuts the ball's soft shaded edge -- rather than proportional to the
     *       radius. So the comparison that matches the noise model is absolute.</li>
     *   <li>Dividing by {@code pred} systematically favours the <b>larger</b> candidate, because
     *       the same absolute error is a smaller fraction of a bigger number. That is a standing
     *       bias toward calling every ball a NECTAR, which is precisely the failure the contact
     *       shadow already causes. Compounding the two would be bad.</li>
     * </ul>
     * The accept/reject gate in the detector is still <i>relative</i>, which is correct there: a
     * tolerance on "is this plausibly a ball at all" should scale with the expected size.
     *
     * @param diametersIn candidate ball diameters, e.g. {@code {2.8, 3.6}}
     * @return the best guess, or null if the pixel is above the horizon
     */
    public static TypeGuess classifyByRadius(double u, double v, double radiusPx,
                                             Intrinsics intr, CameraPose pose,
                                             double[] diametersIn) {
        int n = diametersIn.length;
        double[] preds = new double[n];
        Position[] positions = new Position[n];
        int bestIdx = -1;
        double bestAbsErr = Double.MAX_VALUE;

        for (int i = 0; i < n; i++) {
            Position p = groundPlanePosition(u, v, intr, pose, diametersIn[i]);
            if (p == null) return null;                 // above the horizon
            double pred = predictedRadiusPx(p.rangeIn, intr, pose, diametersIn[i]);
            preds[i] = pred;
            positions[i] = p;
            if (Double.isNaN(pred) || pred <= 0) continue;
            double absErr = Math.abs(pred - radiusPx);
            if (absErr < bestAbsErr) {
                bestAbsErr = absErr;
                bestIdx = i;
            }
        }
        if (bestIdx < 0) return null;

        // Separation: how far the chosen candidate's predicted radius sits from the NEAREST other
        // candidate's. This is a property of the geometry, not of the measurement -- it says how
        // distinguishable the types are here at all.
        double separation = Double.POSITIVE_INFINITY;
        for (int i = 0; i < n; i++) {
            if (i == bestIdx || Double.isNaN(preds[i]) || preds[i] <= 0) continue;
            separation = Math.min(separation, Math.abs(preds[bestIdx] - preds[i]));
        }

        double relErr = Math.abs(preds[bestIdx] - radiusPx) / preds[bestIdx];
        return new TypeGuess(bestIdx, diametersIn[bestIdx], positions[bestIdx],
                preds[bestIdx], relErr, separation);
    }

    /**
     * Ratio of the apparent-size range to the ground-plane range. 1.0 means they agree.
     *
     * <p>Large disagreement means one of three things, and all three are worth knowing:
     * the object is <b>airborne</b> (so the floor assumption is false and the apparent-size answer
     * is the correct one); it is <b>not a ball</b> (a round thing of a different real size); or the
     * radius is badly measured (a blown-out highlight, or two balls merged into one blob).
     *
     * @return the ratio, or NaN if either method has no answer
     */
    public static double consistencyRatio(double u, double v, double radiusPx, Intrinsics intr,
                                          CameraPose pose, double ballDiameterIn) {
        Position gp = groundPlanePosition(u, v, intr, pose, ballDiameterIn);
        double sz = apparentSizeGroundRange(radiusPx, intr, pose, ballDiameterIn);
        if (gp == null || Double.isNaN(sz) || gp.rangeIn <= 0) return Double.NaN;
        return sz / gp.rangeIn;
    }

    /**
     * Robot-frame point to field frame, given the robot pose <b>at the frame's capture time</b>.
     *
     * <p>Use the pose from the capture timestamp, not the newest one. At 50 in/s with 100 ms of
     * vision latency the robot has moved 5 inches since the shutter opened, and all of that error
     * lands directly on the ball's computed field position.
     *
     * @return {@code {fieldX, fieldY}}
     */
    public static double[] robotToField(double xIn, double yIn,
                                        double robotX, double robotY, double robotHeadingRad) {
        double c = Math.cos(robotHeadingRad), s = Math.sin(robotHeadingRad);
        return new double[] { robotX + xIn * c - yIn * s, robotY + xIn * s + yIn * c };
    }

    /**
     * Inches of range error caused by one pixel of measurement error, for both methods.
     *
     * <p>Use it to choose an operating envelope and a sane maximum detection range rather than
     * guessing one.
     * <pre>
     *   ground plane:   |dd/dv| = d^2 / (fy * H)
     *   apparent size:  |dL/dr| = L^2 / (fx * R)
     * </pre>
     *
     * @return {@code {groundPlaneInPerPx, apparentSizeInPerPx}}
     */
    public static double[] rangeErrorPerPixel(double rangeIn, Intrinsics intr,
                                              CameraPose pose, double ballDiameterIn) {
        double H = pose.heightIn - ballDiameterIn / 2.0;
        double R = ballDiameterIn / 2.0;
        double gp = H > 0 ? rangeIn * rangeIn / (intr.fy * H) : Double.POSITIVE_INFINITY;
        double L = Math.hypot(rangeIn, H);
        double sz = L * L / (intr.fx * R);
        return new double[] { gp, sz };
    }

    /**
     * Image row of the horizon, for the overlay and for a quick reject before any contour work.
     *
     * <p>From {@code rz = -sin p - b cos p = 0}, so {@code b = -tan p} and
     * {@code v = cy - fy*tan(pitch)}. Anything above this row cannot be a ball on the floor.
     */
    public static double horizonRow(Intrinsics intr, CameraPose pose) {
        return intr.cy - intr.fy * Math.tan(pose.pitchRad);
    }
}
