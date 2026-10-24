package org.firstinspires.ftc.teamcode;

/**
 * Every number you will ever need to tune lives in this one file.
 *
 * <h2>Coordinate conventions used by ALL of our code</h2>
 * We deliberately use the same convention the FTC SDK and the goBILDA Pinpoint use, so that
 * nothing ever needs a sign flip:
 * <ul>
 *   <li><b>+X is forward</b></li>
 *   <li><b>+Y is to the left</b> (strafe left)</li>
 *   <li><b>+heading is counter-clockwise</b>, 0 rad means "pointing along +X"</li>
 *   <li><b>+Z is up</b></li>
 * </ul>
 * AprilTag {@code ftcPose.bearing} follows the same rule: positive bearing means the target is
 * to our LEFT. (Confirmed from the SDK sample RobotAutoDriveToAprilTagOmni, which feeds bearing
 * straight into a counter-clockwise turn.)
 *
 * <h2>Units</h2>
 * <b>Inches</b> and <b>radians</b> internally. Telemetry converts to degrees for humans.
 * The Pinpoint natively speaks millimetres, so {@link org.firstinspires.ftc.teamcode.subsystems.Odometry}
 * asks it for inches explicitly on every call.
 *
 * <p>Anything marked {@code TUNE ME} is a placeholder. The code will run with these values but the
 * robot will not be accurate until you measure them. Each one says which OpMode measures it.
 */
public final class RobotConstants {

    private RobotConstants() {}

    // ==========================================================================================
    // 1. HARDWARE CONFIGURATION NAMES
    //    These strings must match EXACTLY what you type into the Robot Configuration on the
    //    Driver Station. Lowercase with underscores is the convention.
    // ==========================================================================================
    public static final class Names {
        // -- Drivetrain: Control Hub motor ports 0-3 --
        // These match the team's EXISTING Robot Configuration exactly (lowercase, no separators).
        // The physical configuration wins over any naming convention -- renaming devices on the
        // Driver Station to match a style guide is a great way to break working code the night
        // before a competition.
        public static final String FRONT_LEFT   = "leftfront";
        public static final String FRONT_RIGHT  = "rightfront";
        public static final String BACK_LEFT    = "leftback";
        public static final String BACK_RIGHT   = "rightback";

        /**
         * The goBILDA Pinpoint, as it is ACTUALLY named in the team's robot configuration.
         *
         * <p><b>It is "odomcomp", not "pinpoint".</b> Read straight out of the Hub's own
         * {@code /sdcard/FIRST/firstconfig.xml}:
         * <pre>&lt;goBILDAPinpoint name="odomcomp" port="0" bus="1" /&gt;</pre>
         * An earlier version of this file guessed "pinpoint", which would have thrown at init in
         * every odometry OpMode with a message saying the device was missing -- sending you to look
         * at the I2C cable instead of at a string.
         *
         * <p>Good news from the same file: it is already on <b>bus 1</b>, which is correct. Bus 0 is
         * taken by {@code ControlHubImuBHI260AP name="imu"} -- the Control Hub's built-in IMU -- which
         * is exactly why the Pinpoint must not go there.
         */
        public static final String PINPOINT      = "odomcomp";

        // -- Shooter / turret --
        /**
         * NOTE THE CAPITAL F. The Robot Configuration has this device as "Flywheel", and
         * {@code hardwareMap.get} is <b>case-sensitive</b> -- asking for "flywheel" throws
         * {@code IllegalArgumentException} at init with a message that just says the device is
         * missing, which sends you looking at wiring rather than at spelling.
         *
         * <p>This one is on the <b>Expansion Hub</b>, not the Control Hub. That is fine, but it
         * means its reads travel over RS485 and cost slightly more latency than a Control Hub port,
         * which is worth knowing when you start timing shots.
         */
        public static final String FLYWHEEL      = "Flywheel";
        public static final String FLYWHEEL_2    = "flywheel_2";   // set USE_DUAL_FLYWHEEL=false if absent
        public static final String TURRET        = "turret";       // not configured yet
        public static final String FEEDER        = "transfer";

        // -- Intake / sorting -- (existing config names)
        public static final String INTAKE        = "intakemain";
        public static final String INTAKE_SERVO_L = "leftintake";
        public static final String INTAKE_SERVO_R = "rightintake";
        public static final String BALL_COLOR    = "ball_color";   // REV Color Sensor V3 in the hopper throat

        // -- Vision --
        /** Turret/AprilTag camera. Points at the HIVE. */
        public static final String WEBCAM        = "Webcam 1";
        /**
         * Intake/ball camera. Points down and forward at the floor.
         *
         * <p>Two cameras is the right answer here rather than a shortcut: the tag camera must look
         * up at the HIVE while the ball camera looks down at the floor, so one camera physically
         * cannot do both. If you only have one for now, see {@link org.firstinspires.ftc.teamcode.vision.BallTracker}
         * for the single-camera arrangement.
         */
        public static final String WEBCAM_BALLS  = "Webcam 2";

        /**
         * Limelight 3A. It is not added from the device-type dropdown -- run a <b>scan</b> in the
         * Robot Configuration and it appears as <b>"Ethernet Device"</b>, which you rename to this.
         *
         * <p>Wiring: one USB-C to USB-A cable into the Control Hub's <b>USB 3.0 (blue) Type-A</b>
         * host port. No separate power -- it is bus-powered at 4 W. The USB-C port on the Hub is a
         * <i>device</i> port for a PC and cannot host it. A Control Hub supports only ONE Limelight,
         * and if you add a webcam it goes on the remaining USB 2.0 port.
         *
         * <p>Watch out for the two different IP addresses: the small grey text under the device name
         * in the config is the <b>Control Hub's</b> DHCP-assigned address, not the Limelight's. The
         * IP field in the config wants the Limelight's own, which defaults to the same subnet with
         * last octet 1. Confusing these is the classic setup failure.
         */
        public static final String LIMELIGHT     = "limelight";
    }

    // ==========================================================================================
    // 2. ODOMETRY  (goBILDA 4-Bar Odometry Pack: 2 pods + Pinpoint computer)
    // ==========================================================================================
    public static final class Odo {
        /**
         * Distance from the robot's tracking point to each pod, in MILLIMETRES.
         * The Pinpoint wants mm, so we keep these in mm to avoid a conversion mistake.
         *
         * <p>X_POD_OFFSET: how far SIDEWAYS the X (forward-facing) pod sits.
         *    Left of the tracking point is POSITIVE, right is NEGATIVE.
         * <p>Y_POD_OFFSET: how far FORWARD the Y (sideways-facing) pod sits.
         *    Forward of the tracking point is POSITIVE, backward is NEGATIVE.
         *
         * <p>The "tracking point" is whatever point on the robot you want the reported pose to
         * describe. Use the centre of rotation of the drivetrain -- for a square mecanum chassis
         * that is the geometric centre.
         *
         * <p>TUNE ME: measure with a ruler first, then refine with {@code T2_PinpointOffsetTuner}.
         */
        public static final double X_POD_OFFSET_MM = 72.4;
        public static final double Y_POD_OFFSET_MM = -44.5;

        /**
         * Which way each pod counts. The X pod must count UP when the robot is pushed FORWARD.
         * The Y pod must count UP when the robot is pushed to the LEFT.
         * TUNE ME: {@code T1_OdometryTest} tells you if either is backwards.
         */
        public static final boolean REVERSE_X_ENCODER = false;
        public static final boolean REVERSE_Y_ENCODER = false;

        /**
         * Does the Pinpoint report velocity in the FIELD frame or the ROBOT frame?
         *
         * <p>This matters enormously for shooting on the move, and goBILDA's own docs are
         * ambiguous (they label position "X (forward)" even though position is clearly a field
         * coordinate). Rather than guess, run {@code T3_VelocityFrameTest} -- it answers the
         * question on your actual robot in about 20 seconds -- and set this flag to match.
         *
         * <p>If true, {@link org.firstinspires.ftc.teamcode.subsystems.Odometry} rotates the
         * velocity by the current heading to produce field velocity.
         *
         * <p><b>Now set to false, on third-party evidence.</b> Pedro Pathing's own
         * {@code PinpointLocalizer} feeds the Pinpoint's {@code getVelX()}/{@code getVelY()}
         * straight into its <i>field-frame</i> velocity slot with no rotation applied. Pedro is a
         * widely used, actively maintained library whose path following would visibly break if that
         * were wrong, so it is strong evidence the Pinpoint reports field-relative velocity.
         *
         * <p>Still run {@code T3_VelocityFrameTest} to confirm it on your robot. Third-party
         * evidence is not the same as a measurement, this is load-bearing for the turret, and the
         * test takes twenty seconds.
         */
        public static final boolean VELOCITY_IS_ROBOT_RELATIVE = false;

        /**
         * How long we keep pose history, for matching a camera frame to the pose at the moment
         * the shutter opened. 0.5 s is plenty -- webcam latency is typically 30-100 ms.
         */
        public static final double POSE_HISTORY_SECONDS = 0.5;
    }

    // ==========================================================================================
    // 3. DRIVETRAIN
    // ==========================================================================================
    public static final class Drive {
        /**
         * Flip these if a wheel spins the wrong way.
         *
         * <p><b>All four were inverted after the 2026-10-11 rewire</b> -- pushing the stick forward
         * drove the robot backward, cleanly, with no veer. A clean whole-robot inversion like that
         * means every motor is backwards, not just one side, so all four flags flipped together.
         *
         * <p>Worth understanding why that single change fixes three symptoms at once: with every
         * wheel reversed, the robot's response to forward, strafe <i>and</i> rotation are all
         * inverted together. So if forward was backward, strafe and rotation were backward too --
         * and flipping all four corrects all three in one go. (This is also why "reversed on the
         * wrong side" can never explain a rotation-only fault.)
         *
         * <p>If instead ONE wheel is wrong, the robot veers or spins rather than cleanly reversing.
         * In that case flip only that corner.
         */
        public static final boolean REVERSE_FRONT_LEFT  = true;
        public static final boolean REVERSE_BACK_LEFT   = true;
        public static final boolean REVERSE_FRONT_RIGHT = false;
        public static final boolean REVERSE_BACK_RIGHT  = false;

        /** Deadband on the joysticks, to stop drift from a worn gamepad. */
        public static final double STICK_DEADBAND = 0.05;

        /** Multiplier applied when the driver holds the slow-mode trigger. */
        public static final double SLOW_MODE_SCALE = 0.35;

        /** P gain for the "snap to heading" helper, in power per radian of error. TUNE ME. */
        public static final double HEADING_LOCK_KP = 1.2;
        public static final double HEADING_LOCK_MAX = 0.5;

        // ---- Drivetrain geometry, from the actual hardware ----------------------------------
        // 4x goBILDA 5203-2402-0019 (19.2:1, 312 RPM) on goBILDA 104 mm mecanum wheels.

        /** Encoder counts per revolution of a drive wheel. goBILDA's published figure for 19.2:1. */
        public static final double TICKS_PER_REV = 537.7;

        /**
         * Wheel diameter, inches. <b>104 mm is 4.094488 in, not 4.0.</b> Using 4.0 makes every
         * distance read 2.4% short, which is exactly the kind of error that looks like bad tuning.
         *
         * <p>Expect the effective rolling diameter under a loaded robot to be 1-3 mm UNDER nominal
         * with soft rollers, so verify against a measured 120 in push test rather than trusting
         * the nominal number.
         */
        public static final double WHEEL_DIAMETER_IN = 4.094488;

        public static final double WHEEL_CIRCUMFERENCE_IN = Math.PI * WHEEL_DIAMETER_IN; // 12.863
        public static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_IN; // 41.80
        public static final double INCHES_PER_TICK = WHEEL_CIRCUMFERENCE_IN / TICKS_PER_REV; // 0.02392

        /**
         * Theoretical free speed, in/s, at 312 RPM. <b>An upper bound and a sanity check only</b> --
         * a real robot reaches roughly 80-85% of it. Expect to measure 53-58 in/s forward and
         * 42-46 in/s strafing. Pedro Pathing's translational constants must be MEASURED by its
         * tuners, never computed from this.
         */
        public static final double FREE_SPEED_IN_PER_SEC = 66.889;
    }

    // ==========================================================================================
    // 4. SHOOTER GEOMETRY AND BALLISTICS
    // ==========================================================================================
    public static final class Shoot {
        /**
         * Where the ball actually leaves the robot, relative to the odometry tracking point,
         * in INCHES, in the robot frame (+X forward, +Y left, +Z up from the floor).
         * Measure to the point where the ball loses contact with the flywheel.
         * TUNE ME (tape measure).
         */
        public static final double MUZZLE_X_IN = 2.0;
        public static final double MUZZLE_Y_IN = 0.0;
        public static final double MUZZLE_Z_IN = 14.0;

        /**
         * Fixed launch angle of the hood, above horizontal, in DEGREES.
         * Most FTC shooters have a fixed hood; if yours is adjustable, see
         * {@link org.firstinspires.ftc.teamcode.util.Ballistics} for how to extend the solver.
         *
         * <p>Physics note: a higher angle needs more speed but gives a steeper, more forgiving
         * entry into the CELL. 35-45 deg is the usual sweet spot for a ball that must drop into
         * an upward-facing opening. TUNE ME.
         */
        public static final double LAUNCH_PITCH_DEG = 40.0;

        /** Flywheel outside diameter in inches, INCLUDING any compliant wheel tread. TUNE ME. */
        public static final double FLYWHEEL_DIAMETER_IN = 4.0;

        /**
         * FALSE: this team has exactly ONE shooter motor (goBILDA 5203-2402-0001, 1:1, 6000 RPM).
         *
         * <p>Leaving this true throws {@code IllegalArgumentException} out of Shooter's constructor
         * at OpMode <b>init</b> -- the OpMode dies before START, which reads like a config problem
         * rather than a constants problem.
         */
        public static final boolean USE_DUAL_FLYWHEEL = false;

        /**
         * Which way the flywheel spins. Flip this if the ball is thrown the wrong way.
         *
         * <p>Nothing was setting a direction on the flywheel before, so it ran whichever way the
         * motor happened to be wired -- and on this robot that was backwards.
         *
         * <p>Change it <b>here</b>, not by flipping signs at call sites. The shooter is commanded by
         * RPM and {@link #MAX_FLYWHEEL_RPM} clamps at 0 on the low end, so a negative "fix" applied
         * somewhere else would clamp straight back to zero and look like a dead motor.
         *
         * <p>If you add a second flywheel later, it is driven to the OPPOSITE direction
         * automatically -- opposed wheels have to counter-rotate to throw the ball forward.
         */
        public static final boolean REVERSE_FLYWHEEL = true;

        /**
         * Encoder counts per revolution of the flywheel. <b>28.0 is correct as written</b> for this
         * team's motor.
         *
         * <p>Every goBILDA Yellow Jacket puts its encoder on the <b>bare motor shaft, before the
         * gearbox</b>, at 28 counts/rev. goBILDA's published "PPR at the Output Shaft" is just
         * {@code 28 x exact gear ratio}. The shooter is 1:1, so output PPR = 28. If you ever belt
         * the flywheel to a different speed, divide by the belt ratio.
         *
         * <p>A useful consequence: every 5203, whatever its gearbox, free-runs its encoder at about
         * <b>2800 ticks/sec</b>, because the motor shaft always turns at ~6000 RPM. The Control Hub
         * reports velocity in a signed 16-bit field (+/-32767 ticks/sec), so there is 11.7x
         * headroom -- closed-loop RPM control on a 6000 RPM motor is perfectly safe. The real
         * limitation is the opposite: 28 counts/rev is the <b>coarsest velocity feedback on the
         * robot</b>, which is why {@link #RPM_TOLERANCE} is generous and why D is useless here.
         */
        public static final double FLYWHEEL_TICKS_PER_REV = 28.0;

        /**
         * Velocity PIDF for the flywheel.
         *
         * <p>F and P live on the same "32767 = full output" scale, so F is derived, not guessed:
         * {@code F = 32767 / maxTicksPerSecond = 32767 / 2800 = 11.70}. P belongs in the
         * {@code 0.1F .. F/3} band, so ~1.2 to ~3.9.
         *
         * <p>Leave <b>I at 0</b>: spin-up takes over a second on a real flywheel and any integral
         * term winds up hard and overshoots. Leave <b>D at 0</b>: at 28 counts/rev the velocity
         * signal is quantised coarsely enough that differentiating it produces noise, not damping.
         *
         * <p>Before overriding these at all, print {@code getPIDFCoefficients(RUN_USING_ENCODER)} --
         * the SDK ships tuned PIDF defaults for goBILDA motor types and they may beat these.
         */
        public static final double FLYWHEEL_P = 1.17;   // 0.1 * F
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.0;
        public static final double FLYWHEEL_F = 11.70;  // 32767 / 2800

        /**
         * Flywheel is "ready" when within this many RPM of target for READY_STABLE_SEC.
         *
         * <p>There is a genuine tension in this number and it is worth knowing about:
         * <ul>
         *   <li><b>Accuracy</b> wants it tight. Range changes about <b>0.117 in per RPM</b>, so
         *       40 RPM of error is ~4.7 in of range -- roughly half a CELL opening.</li>
         *   <li><b>Measurability</b> wants it loose. With only 28 counts/rev the Hub's velocity
         *       estimate is quantised in steps that can approach 100 RPM over a short sample
         *       window, so too tight a tolerance makes {@code isAtSpeed()} chatter or never latch
         *       at all -- and a gate that never closes is worse than a loose one.</li>
         * </ul>
         * 100 splits the difference. {@link org.firstinspires.ftc.teamcode.subsystems.Shooter}
         * averages several velocity samples to claw back some of the quantisation, which is what
         * makes a number this tight usable. Confirm on the real robot: if READY flickers, raise it.
         */
        public static final double RPM_TOLERANCE = 100.0;
        public static final double READY_STABLE_SEC = 0.15;

        /** Number of velocity samples to average, to fight 28-counts/rev quantisation. */
        public static final int RPM_SAMPLE_COUNT = 5;

        /**
         * Safety clamp. Free speed is 6000 RPM, but the whole game lives in roughly
         * <b>1600-2900 RPM</b> on a 4 in wheel -- a 14 ft shot needs only about 7.3 m/s of ball
         * exit speed. Keeping the clamp well above the useful band is fine; it exists to catch a
         * runaway ballistics solution, not to define the operating point.
         */
        public static final double MAX_FLYWHEEL_RPM = 5000.0;

        /**
         * Ramp the commanded RPM rather than stepping it. A step draws close to the motor's 9.2 A
         * stall current for over a second, and doing that while four drive motors are accelerating
         * is how you brown out the Control Hub mid-match. 0 disables ramping.
         */
        public static final double RPM_RAMP_PER_SEC = 4000.0;
    }

    // ==========================================================================================
    // 5. TURRET
    // ==========================================================================================
    public static final class Turret {
        /**
         * Encoder ticks per radian of turret rotation.
         *
         * <p>The turret motor is a goBILDA 5203-2402-0019 (19.2:1, 312 RPM), published as
         * <b>537.7 PPR</b> at its output shaft. With an external reduction of {@code G} between
         * that output shaft and turret rotation:
         * <pre>
         *   TICKS_PER_RAD = 537.7 * G / (2*pi) = 85.5777 * G
         * </pre>
         * Worked values: G=1 -> 85.6 (unusably coarse, 1.5 ticks/deg), <b>G=5 -> 427.9</b>,
         * G=6.25 -> 534.9, G=8 -> 684.6, G=10 -> 855.8, G=20 -> 1711.6 (too slow, 94 deg/s).
         *
         * <p>Free slew speed is {@code 1872 / G} deg/s, so G in the <b>5 to 8</b> band gives
         * 234-374 deg/s and 122-194 kg.cm of holding torque, which is the right place to be.
         * <b>Direct drive (G=1) is not viable.</b>
         *
         * <p>The 427.9 below assumes G=5. <b>Measure it rather than trusting a nameplate ratio:</b>
         * zero the encoder, rotate the turret exactly 90 degrees against a protractor, read the
         * ticks, and compute {@code ticks / (pi/2)}. Measuring directly folds in belt slop and any
         * miscounted teeth, and removes one place to make an arithmetic error.
         *
         * <p>Note the previous placeholder here was 1200.0, which silently implied G=14.02 -- almost
         * certainly not what you built. Also be aware the real accuracy limit is <b>backlash</b>,
         * not encoder resolution, and each extra gear stage adds its own; prefer ONE clean external
         * reduction over two. At G=5, full +/-170 deg of travel is 4.7 turns of the motor output
         * shaft, so route the turret wiring with a service loop or it will tear out.
         */
        public static final double TICKS_PER_RAD = 427.889;   // 85.5777 * G, with G = 5

        /** Mechanical travel limits in radians, relative to "straight forward" = 0. */
        public static final double MIN_ANGLE_RAD = Math.toRadians(-170);
        public static final double MAX_ANGLE_RAD = Math.toRadians(170);

        /** Position PID, power per radian of error. TUNE ME. */
        public static final double KP = 2.4;
        public static final double KI = 0.0;
        public static final double KD = 0.12;

        /** Static friction feedforward -- smallest power that makes the turret creep. TUNE ME. */
        public static final double KS = 0.045;

        /**
         * Feedforward that cancels robot rotation. When the chassis spins counter-clockwise at
         * w rad/s, the turret must spin clockwise at w rad/s just to hold its field aim.
         * Units: power per (rad/s). TUNE ME -- start at KV and adjust until the turret holds a
         * field target while you spin the robot in place.
         */
        public static final double KV = 0.11;

        /** Considered "on target" within this angular error. */
        public static final double TOLERANCE_RAD = Math.toRadians(1.5);

        public static final double MAX_POWER = 0.85;
    }

    // ==========================================================================================
    // 6. CAMERA MOUNT  (for turning an AprilTag sighting into a field coordinate)
    // ==========================================================================================
    public static final class Camera {
        /**
         * Camera lens position relative to the odometry tracking point, robot frame, INCHES.
         * TUNE ME (tape measure).
         */
        public static final double X_IN = 5.0;    // forward of tracking point
        public static final double Y_IN = 0.0;    // left of tracking point
        public static final double Z_IN = 10.5;   // above the floor

        /**
         * Which way the camera points, relative to robot forward, in DEGREES, CCW positive.
         * 0 = straight ahead. If you mount the camera ON the turret, set
         * {@link #MOUNTED_ON_TURRET} true instead and this becomes the offset from turret-zero.
         */
        public static final double YAW_DEG = 0.0;

        /**
         * True if the camera rotates with the turret. A turret-mounted camera keeps the target
         * centred (great for range accuracy) but loses the target whenever the turret slews.
         * A chassis-mounted camera with a wide FOV is easier to make reliable -- start there.
         */
        public static final boolean MOUNTED_ON_TURRET = false;

        /** Stream resolution. Lower = faster and less latency. 640x480 is a good AprilTag default. */
        public static final int WIDTH  = 640;
        public static final int HEIGHT = 480;

        /**
         * Lens intrinsics in pixels at the resolution above, used by the ball tracker to turn
         * pixels into angles and distances. The SDK already knows these for common webcams for
         * AprilTag purposes, but the ball tracker needs them explicitly.
         * TUNE ME: {@code T8_CameraIntrinsics}, or read them off the SDK's built-in calibration
         * for your camera in teamwebcamcalibrations.xml.
         */
        public static final double FX = 622.0;
        public static final double FY = 622.0;
        public static final double CX = 319.5;
        public static final double CY = 239.5;

        /**
         * Manual exposure keeps AprilTags sharp while the robot moves. TUNE ME.
         *
         * <p>These apply only to a <b>UVC webcam</b> via {@code ExposureControl}/{@code GainControl}.
         * Those classes do not exist for a Limelight -- the 3A has a rolling shutter and you set a
         * short fixed exposure and raised gain in the <b>Limelight web UI's pipeline Input tab</b>
         * instead. The need is the same: on auto-exposure, a moving robot smears tags into
         * unreadability exactly when you need them.
         */
        public static final long EXPOSURE_MS = 6;
        public static final int  GAIN        = 250;

        /**
         * Limelight result poll rate, Hz. Valid 1-250, SDK default 100.
         *
         * <p>Must be set <b>before</b> {@code start()} -- {@code setPollRateHz()} silently does
         * nothing if the poller is already running, with no exception and no log. Note the interval
         * is computed with integer division, so rates that do not divide 1000 evenly end up slightly
         * faster than requested.
         */
        public static final int LIMELIGHT_POLL_HZ = 100;
    }

    // ==========================================================================================
    // 7. HIVE TRACKING
    // ==========================================================================================
    public static final class Hive {
        /**
         * The BIOBUZZ CELL AprilTag clusters are named. Leave null to accept ANY cluster, which is
         * the right thing to do until you have read the cluster names off your own field with
         * {@code T4_HiveTrackerTest}. Then set this to your alliance's HIVE cluster short name so
         * you never lock onto the opponent's HIVE.
         */
        public static final String TARGET_CLUSTER_SHORT_NAME = null;

        /**
         * Low-pass filter on the target's estimated field position. 0 = never update (frozen),
         * 1 = trust every frame completely (jittery). 0.35 is a good start.
         */
        public static final double POSITION_FILTER_ALPHA = 0.35;

        /**
         * Reject a sighting if fewer than this percent of the cluster's member tags were found.
         * A 1-of-4 sighting still gives a full 6DOF pose but it is noisier. 25 accepts everything.
         */
        public static final int MIN_PERCENT_CLUSTER_FOUND = 25;

        /**
         * After this long with no sighting we stop trusting the dead-reckoned target estimate.
         * Within this window, odometry carries the aim through the gap.
         */
        public static final double MAX_ESTIMATE_AGE_SEC = 1.5;

        /**
         * Aim point offset above the cluster origin, inches. The cluster origin is already at the
         * centre of the CELL opening in BIOBUZZ, so 0 is correct. Use this only if empirical
         * testing shows you are consistently low or high.
         */
        public static final double AIM_Z_BIAS_IN = 0.0;

        // ---- Tag centroid -> CELL opening centre -------------------------------------------
        // Every cluster member sits at (x, 7.1874, -5.622) inches from the cluster origin, with x
        // in {-6.5, -2.75, +2.75, +6.5}. Those MAGNITUDES came out of the compiled SDK binary and
        // are solid. See BiobuzzTags.
        //
        // What is NOT solid is how the cluster plane's axes map onto our +forward/+left/+up frame,
        // because the SDK does not document it. So the two constants below carry the right
        // magnitudes with a physically-reasoned sign, and are tunable.
        //
        // The reasoning: the tags are on the BOTTOM face of the CELL and the opening faces UP, so
        // the origin should be above the tags. If T4 shows the aim is consistently high or low by
        // about 7 inches, flip UP's sign. If it is consistently near or far by about 5.6 inches,
        // flip FORWARD's. One test tells you both.

        /** Origin offset above the tag centroid, inches. Magnitude verified; sign to confirm. */
        public static final double CELL_OFFSET_UP_IN = 7.1874;

        /** Origin offset beyond the tag centroid, inches. Magnitude verified; sign to confirm. */
        public static final double CELL_OFFSET_FORWARD_IN = 5.622;

        /**
         * Scale on the sideways per-member correction. 1.0 is correct.
         *
         * <p>Set it to 0 to disable the correction entirely as a diagnostic: because the four x
         * offsets are symmetric and sum to zero, seeing all four tags makes this correction a no-op
         * anyway. So if aiming is good with four tags visible and biased sideways with one, this
         * constant -- and the assumption that member order follows tag-ID order -- is what to suspect.
         */
        public static final double CELL_OFFSET_SCALE = 1.0;
    }

    // ==========================================================================================
    // 8. BALL COLOUR CLASSIFICATION
    //    POLLEN is yellow, 2.8 in, 0.055 lb. NECTAR is red or blue, 3.6 in, 0.091 lb.
    // ==========================================================================================
    public static final class Colors {
        /** Hue windows in degrees (0-360). TUNE ME: {@code T6_ColorSensorCalibration} prints hue. */
        public static final double POLLEN_HUE_MIN = 35,  POLLEN_HUE_MAX = 85;   // yellow
        public static final double RED_HUE_MIN    = 340, RED_HUE_MAX    = 20;   // wraps through 0
        public static final double BLUE_HUE_MIN   = 190, BLUE_HUE_MAX   = 265;

        /** Below this saturation we call it "no ball / ambient". TUNE ME. */
        public static final double MIN_SATURATION = 0.35;

        /** A ball is present if the sensor's distance reading is under this, in mm. TUNE ME. */
        public static final double BALL_PRESENT_MM = 25.0;

        /** Gain on the colour sensor. Raise it if saturation readings are weak. */
        public static final float SENSOR_GAIN = 12.0f;
    }

    /** Set this before every match, or read it from the OpMode name. */
    public static volatile Alliance alliance = Alliance.BLUE;

    public enum Alliance { RED, BLUE }
}
