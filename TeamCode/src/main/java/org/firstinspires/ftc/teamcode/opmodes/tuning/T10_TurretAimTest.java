package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayDeque;

/**
 * Turret auto-aim v2: latency-compensated, holds aim between frames using Pinpoint heading.
 *
 * START WITH THE TURRET POINTING STRAIGHT AHEAD AND THE ROBOT STILL (encoder zeroes, IMU calibrates).
 *
 * Hold LEFT BUMPER = auto-aim. Release = hold.
 * Right stick X = manual. Y = toggle camera exposure (short / auto) for comparison.
 */
@TeleOp(name = "T10 Turret Aim v2", group = "Test")
public class T10_TurretAimTest extends OpMode {

  private static final double AIMED_DEG = 1.5;
  private static final double MANUAL_POWER = 0.3;
  private static final double STICK_DEADBAND = 0.1;
  private static final long HISTORY_NANOS = 500_000_000L;   // keep 0.5 s of history

  /** One sample of turret angle and robot heading at a moment in time. */
  private static class Sample {
    final long nanos; final double turretRad; final double headingRad;
    Sample(long n, double t, double h) { nanos = n; turretRad = t; headingRad = h; }
  }

  private Turret turret;
  private HiveTracker tracker;
  private GoBildaPinpointDriver pinpoint;

  private final ArrayDeque<Sample> history = new ArrayDeque<>();
  private double headingRad;        // continuous (unwrapped) robot heading
  private double lastRawHeading;
  private double targetRad = 0;     // turret target, robot frame

  private long lastFrameNano = 0;
  private long lastSeenMs = 0;
  private double lastBearingDeg = 0;
  private String lastCluster = "none";
  private boolean shortExposure = true;
  private boolean prevY = false;
  private boolean manual = false;

  @Override
  public void init() {
    turret = new Turret(hardwareMap);            // zeroes: must point straight ahead
    turret.setTargetAngleRad(0);

    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.Names.PINPOINT);
    pinpoint.resetPosAndIMU();                   // robot must be still
    pinpoint.update();
    lastRawHeading = pinpoint.getHeading(AngleUnit.RADIANS);
    headingRad = 0;

    tracker = new HiveTracker(hardwareMap);
    tracker.setManualExposure();                 // short exposure: less motion blur

    telemetry.addLine("Ready. Turret straight ahead, robot still. Hold LB to aim, Y = exposure.");
    telemetry.update();
  }

  /** Turret angle and heading at a past moment (nearest recorded sample). */
  private Sample sampleAt(long nanos) {
    Sample best = null;
    for (Sample s : history) {
      if (best == null || Math.abs(s.nanos - nanos) < Math.abs(best.nanos - nanos)) best = s;
    }
    return best;
  }

  @Override
  public void loop() {
    long now = System.nanoTime();

    // ---- Robot heading from the Pinpoint (unwrapped, so +/-180 crossings don't jump) ----
    pinpoint.update();
    double raw = pinpoint.getHeading(AngleUnit.RADIANS);
    double dHeading = Math.atan2(Math.sin(raw - lastRawHeading), Math.cos(raw - lastRawHeading));
    lastRawHeading = raw;
    headingRad += dHeading;
    double headingVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

    // ---- Record history for latency compensation ----
    history.addLast(new Sample(now, turret.getAngleRad(), headingRad));
    while (!history.isEmpty() && now - history.peekFirst().nanos > HISTORY_NANOS) history.removeFirst();

    // ---- Exposure toggle (Y) ----
    if (gamepad1.y && !prevY) {
      shortExposure = !shortExposure;
      if (shortExposure) {
        tracker.setManualExposure();
      } else {
        ExposureControl exp = tracker.getVisionPortal().getCameraControl(ExposureControl.class);
        if (exp != null) exp.setMode(ExposureControl.Mode.Auto);
      }
    }
    prevY = gamepad1.y;

    // ---- Best HIVE cluster in the newest detections ----
    AprilTagClusterDetection best = null;
    for (AprilTagDetection d : tracker.getRawDetections()) {
      if (!(d instanceof AprilTagClusterDetection)) continue;
      AprilTagClusterDetection c = (AprilTagClusterDetection) d;
      if (c.metadata == null || c.ftcPose == null) continue;
      if (c.percentClusterFound < RobotConstants.Hive.MIN_PERCENT_CLUSTER_FOUND) continue;
      String wanted = RobotConstants.Hive.TARGET_CLUSTER_SHORT_NAME;
      if (wanted != null && !wanted.equals(c.metadata.shortName)) continue;
      if (best == null || c.percentClusterFound > best.percentClusterFound) best = c;
    }
    boolean newFrame = best != null && best.frameAcquisitionNanoTime != lastFrameNano;
    if (best != null) {
      lastBearingDeg = best.ftcPose.bearing;
      lastCluster = best.metadata.shortName;
      lastSeenMs = System.currentTimeMillis();
    }
    boolean seenRecently = System.currentTimeMillis() - lastSeenMs < 300;

    boolean aiming = gamepad1.left_bumper;
    double stick = gamepad1.right_stick_x;
    String mode;

    if (Math.abs(stick) > STICK_DEADBAND && !aiming) {
      // ---- Manual ----
      manual = true;
      double angle = turret.getAngleRad();
      double power = -stick * MANUAL_POWER;
      if (angle >= RobotConstants.Turret.MAX_ANGLE_RAD && power > 0) power = 0;
      if (angle <= RobotConstants.Turret.MIN_ANGLE_RAD && power < 0) power = 0;
      turret.setRawPower(power);
      mode = "MANUAL";
    } else {
      if (manual) { targetRad = turret.getAngleRad(); manual = false; }

      if (aiming) {
        // Fix B: hold the aim in the FIELD between frames by countering chassis rotation.
        targetRad -= dHeading;

        // Fix A: on a new frame, compute the target from where the turret and robot were
        // when that frame was CAPTURED, then account for rotation since then.
        if (newFrame) {
          Sample then = sampleAt(best.frameAcquisitionNanoTime);
          if (then != null) {
            double bearingRad = Math.toRadians(best.ftcPose.bearing);
            targetRad = then.turretRad + bearingRad - (headingRad - then.headingRad);
          }
          lastFrameNano = best.frameAcquisitionNanoTime;
        }
      }

      // Clip, so an unreachable target behind the robot doesn't wind up beyond the limits.
      targetRad = Math.max(RobotConstants.Turret.MIN_ANGLE_RAD,
          Math.min(RobotConstants.Turret.MAX_ANGLE_RAD, targetRad));
      turret.setTargetAngleRad(targetRad);
      turret.update(aiming ? headingVel : 0);   // rotation feedforward while aiming

      if (!aiming) mode = "HOLD";
      else if (!seenRecently) mode = "AIM: NO TARGET (holding field direction)";
      else if (Math.abs(lastBearingDeg) <= AIMED_DEG) mode = "AIM: ON TARGET";
      else mode = "AIM: tracking";
    }

    telemetry.addData("Mode", mode);
    telemetry.addData("Camera FPS", "%.1f", tracker.getVisionPortal().getFps());
    telemetry.addData("Exposure (Y)", shortExposure ? "SHORT" : "AUTO");
    telemetry.addData("Cluster", seenRecently ? lastCluster : "none");
    telemetry.addData("Bearing (deg)", seenRecently ? String.format("%+.1f", lastBearingDeg) : "-");
    telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDeg());
    telemetry.addData("Robot heading (deg)", "%.1f", Math.toDegrees(headingRad));
    telemetry.update();
  }

  @Override
  public void stop() {
    turret.stop();
    tracker.close();
  }
}