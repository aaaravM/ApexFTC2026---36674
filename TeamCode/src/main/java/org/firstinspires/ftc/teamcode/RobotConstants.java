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

        // -- Odometry: configure as "goBILDA Pinpoint Odometry Computer" on an I2C bus.
        //    IMPORTANT: use I2C bus 1, 2 or 3 -- NOT bus 0. Bus 0 shares with the Control Hub's
        //    internal IMU and you will get bus contention / dropped reads.
        public static final String PINPOINT      = "pinpoint";

        // -- Shooter / turret -- (not yet in the team's configuration; add these as you build)
        public static final String FLYWHEEL      = "flywheel";
        public static final String FLYWHEEL_2    = "flywheel_2";   // set USE_DUAL_FLYWHEEL=false if absent
        public static final String TURRET        = "turret";
        public static final String FEEDER        = "feeder";       // servo or CR servo that pushes a ball in

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
        public static final double X_POD_OFFSET_MM = -84.0;
        public static final double Y_POD_OFFSET_MM = -168.0;

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
         */
        public static final boolean VELOCITY_IS_ROBOT_RELATIVE = true;

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
        /** Flip these if a wheel spins the wrong way on the very first test. */
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

        /** Set false if you only have one flywheel motor. */
        public static final boolean USE_DUAL_FLYWHEEL = true;

        /**
         * Motor encoder counts per revolution of the FLYWHEEL (not the motor, if geared).
         * goBILDA 5202/5203 bare motor = 28 counts/rev at the motor shaft.
         * A 1:1 direct-drive bare 6000 rpm motor is therefore 28. If you belt it 1:2 to speed
         * the wheel up, divide by 2. TUNE ME.
         */
        public static final double FLYWHEEL_TICKS_PER_REV = 28.0;

        /** Velocity PIDF for the flywheel. F dominates; start with F and add a little P. TUNE ME. */
        public static final double FLYWHEEL_P = 50.0;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.0;
        public static final double FLYWHEEL_F = 14.0;

        /** Flywheel is "ready" when within this many RPM of target for READY_STABLE_SEC. */
        public static final double RPM_TOLERANCE = 60.0;
        public static final double READY_STABLE_SEC = 0.12;

        /** Safety clamp: never command more than this. Set to your measured free speed. TUNE ME. */
        public static final double MAX_FLYWHEEL_RPM = 5000.0;
    }

    // ==========================================================================================
    // 5. TURRET
    // ==========================================================================================
    public static final class Turret {
        /** Encoder ticks per radian of turret rotation. TUNE ME: {@code T7_TurretTuner}. */
        public static final double TICKS_PER_RAD = 1200.0;

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

        /** Manual exposure keeps AprilTags sharp while the robot moves. TUNE ME. */
        public static final long EXPOSURE_MS = 6;
        public static final int  GAIN        = 250;
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
