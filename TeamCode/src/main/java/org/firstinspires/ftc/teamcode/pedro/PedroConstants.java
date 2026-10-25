package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Builds the Pedro Pathing {@link Follower}. One place, used by every OpMode.
 *
 * <h2>This is Pedro Pathing 3.0.1 (published 2026-09-18) and v3 is a FULL REWRITE</h2>
 * Almost every Pedro tutorial, video and forum post you will find predates it and <b>will not
 * compile</b>. The traps, in the order you are likely to hit them:
 *
 * <ul>
 *   <li><b>The dependency name changed.</b> v2 was {@code com.pedropathing:ftc}, which is frozen at
 *       2.1.2 and still resolves from Maven Central. So pasting a v2 tutorial's Gradle line fails
 *       <i>silently</i> -- you get the whole v2 API and spend an evening writing {@code
 *       FollowerBuilder} code against a version you did not mean to install. v3 is
 *       {@code com.pedropathing:revhub:3.0.1}.</li>
 *   <li><b>There is no {@code FollowerBuilder} and no {@code Constants} builder.</b> You call a
 *       three-arg constructor, and the argument order is <b>(Localizer, Drivetrain, Algorithm)</b>.
 *       Pedro's own Quickstart stub has a comment saying {@code Follower(Drivetrain, Localizer,
 *       Foresight)} -- that comment is wrong. The types are unrelated so a wrong order is a compile
 *       error rather than a silent bug, but don't "fix" it by reordering blindly.</li>
 *   <li><b>Config is a lambda, not chained setters.</b> {@code new MecanumConfig(c -> {
 *       c.frontLeftName.set("leftfront"); ... })}. Each field is a {@code ConfigVar<T>}.</li>
 *   <li><b>{@code Pose} moved</b> to {@code com.pedropathing.math.Pose} (v2:
 *       {@code com.pedropathing.geometry.Pose}). This is the single most common copy-paste failure.</li>
 *   <li><b>{@code turnTo()} is gone</b> with no direct replacement. Use path heading interpolators,
 *       {@code ManualDrive.headingLock(...)}, or {@code follower.hold(pose)}.</li>
 * </ul>
 *
 * <h2>The motor-name trap, which matters to us specifically</h2>
 * v2's {@code leftFrontMotorName}/{@code leftRearMotorName} became v3's {@code frontLeftName}/
 * {@code backLeftName} -- the word order flipped AND "rear" became "back". It reads as cosmetic,
 * which is exactly why it is dangerous: it is easy to wire {@code backLeftName} to the right-rear
 * motor.
 *
 * <p><b>A robot with two motors transposed drives almost correctly in translation and completely
 * wrong in rotation.</b> That is worth reading twice, because it is the symptom we are already
 * chasing. If Pedro drives but will not turn, check these four names against the physical robot
 * before touching anything else.
 */
public final class PedroConstants {

    private PedroConstants() {}

    /**
     * Build a Follower.
     *
     * <p>Call once in {@code runOpMode()} / {@code init()}. The Follower owns the four drive motors
     * and the Pinpoint from this point on -- nothing else may touch them. In particular, do not also
     * construct {@link org.firstinspires.ftc.teamcode.subsystems.Odometry}: see
     * {@link org.firstinspires.ftc.teamcode.subsystems.PedroOdometry} for why, and use that instead.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                buildLocalizer(hardwareMap),   // Localizer FIRST -- see class javadoc
                buildDrivetrain(hardwareMap),
                new Foresight(FollowerTuning.build()));
    }

    // ==========================================================================================
    // Localizer -- goBILDA Pinpoint with the 4-Bar Odometry Pack
    // ==========================================================================================
    private static PinpointLocalizer buildLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig());
    }

    /** The Pinpoint settings, shared by the Follower and the AutoTune procedures. */
    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set(RobotConstants.Names.PINPOINT);

            // ---- THE OFFSET CONVENTION, settled ----------------------------------------------
            // Pedro passes these straight through to the goBILDA driver:
            //     pinpoint.setOffsets(xPodOffset, yPodOffset, offsetUnits)
            // with NO swap and NO negation. So xPodOffset IS goBILDA's xOffset and yPodOffset IS
            // goBILDA's yOffset, and our measured numbers copy across verbatim.
            //
            // But the NAMES are actively misleading, so be clear about what they mean:
            //   xPodOffset = how far SIDEWAYS the X (forward-reading) pod sits.  LEFT is positive.
            //   yPodOffset = how far FORWARD the Y (strafe-reading) pod sits.    FORWARD is positive.
            // Neither is "a distance along the X axis". Getting this wrong does not crash and does
            // not affect straight-line driving -- only pose drift when the robot rotates, which is
            // exactly what T2_PinpointOffsetTuner exists to catch.
            c.xPodOffset.set(RobotConstants.Odo.X_POD_OFFSET_MM);
            c.yPodOffset.set(RobotConstants.Odo.Y_POD_OFFSET_MM);

            // REQUIRED. offsetUnits defaults to INCH, but RobotConstants stores mm. Omitting this
            // scales both offsets by 25.4 -- silently, with no crash and no straight-line symptom.
            c.offsetUnits.set(DistanceUnit.MM);

            // Everything Pedro reports comes back in this unit. Inches, to match the rest of the
            // codebase. Stated explicitly rather than relying on the default, because every
            // ballistics number downstream assumes it.
            c.globalDistanceUnit.set(DistanceUnit.INCH);

            // Our hardware is the 4-Bar Odometry Pack. This IS the default, but stating it means a
            // future Pedro release changing its default cannot silently change our pod scaling.
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Pod counting directions, carried over from RobotConstants so there is one source of
            // truth shared with the non-Pedro Odometry class.
            c.xPodDirection.set(RobotConstants.Odo.REVERSE_X_ENCODER
                    ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                    : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(RobotConstants.Odo.REVERSE_Y_ENCODER
                    ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                    : GoBildaPinpointDriver.EncoderDirection.FORWARD);

            // Recalibrate the IMU at construction but keep the pose, so an autonomous OpMode can set
            // a known starting pose afterwards without it being zeroed underneath.
            // The robot MUST be stationary while this happens.
            c.resetMode.set(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);
        });
    }

    /** Boolean "is reversed" to the SDK Direction enum Pedro's config wants. */
    private static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    // ==========================================================================================
    // Drivetrain -- mecanum
    // ==========================================================================================
    private static Mecanum buildDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig());
    }

    /** The drivetrain settings, shared by the Follower and the AutoTune procedures. */
    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            // Names must match the Robot Configuration on the Driver Station exactly.
            // Read the class javadoc about frontLeft/backLeft before editing these.
            c.frontLeftName.set(RobotConstants.Names.FRONT_LEFT);
            c.frontRightName.set(RobotConstants.Names.FRONT_RIGHT);
            c.backLeftName.set(RobotConstants.Names.BACK_LEFT);
            c.backRightName.set(RobotConstants.Names.BACK_RIGHT);

            // Our existing teleop reverses the LEFT pair. Pedro's mecanum mixing is byte-for-byte
            // identical to the mixing already in MecanumTeleOp, so these transfer 1:1 with no sign
            // rework at all.
            //
            // Note these are Direction enums, not booleans -- the field names are
            // *Direction and take DcMotorSimple.Direction, verified against the compiled
            // revhub-3.0.1 jar rather than guessed.
            c.frontLeftDirection.set(direction(RobotConstants.Drive.REVERSE_FRONT_LEFT));
            c.frontRightDirection.set(direction(RobotConstants.Drive.REVERSE_FRONT_RIGHT));
            c.backLeftDirection.set(direction(RobotConstants.Drive.REVERSE_BACK_LEFT));
            c.backRightDirection.set(direction(RobotConstants.Drive.REVERSE_BACK_RIGHT));

            // Pedro's Mecanum sets ZeroPowerBehavior.FLOAT in its constructor and only brakes when
            // this is true AND the call came through manual(). Leave it false and the robot COASTS
            // on stick release -- so odometry still reports velocity after the driver thinks they
            // have stopped, and the turret keeps applying a lead it no longer needs.
            c.manualBrakeMode.set(true);
        });
    }

    // ==========================================================================================
    // Algorithm -- Foresight
    // ==========================================================================================

    // The Foresight tuning numbers live in FollowerTuning, which deliberately has no Android
    // imports so it can be compiled and RUN on a laptop. That is how we verified that all 12
    // required ForesightConfig fields are set and every validator accepts them -- so
    // follower.update() cannot throw "Config variable has not been set". See that class.
}
