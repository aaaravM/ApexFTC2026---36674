package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Turret auto-aim test: turns the turret to centre the HIVE tag cluster in the camera.
 *
 * START WITH THE TURRET POINTING STRAIGHT AHEAD (encoder zeroes at INIT).
 *
 * Hold LEFT BUMPER (gamepad 1) = auto-aim. Release = hold current angle.
 * Right stick X = manual (slow). Software limits always apply.
 */
@TeleOp(name = "T9 Turret Aim Test", group = "Test")
public class T9_TurretAimTest extends OpMode {

  /** Fraction of each camera correction applied. <1 avoids oscillation from camera latency. */
  private static final double AIM_GAIN = 0.7;
  /** Considered aimed when the target is within this many degrees of the image centre. */
  private static final double AIMED_DEG = 1.5;
  private static final double MANUAL_POWER = 0.3;
  private static final double STICK_DEADBAND = 0.1;

  private Turret turret;
  private HiveTracker tracker;

  private long lastFrameNano = 0;
  private long lastSeenMs = 0;
  private double lastBearingDeg = 0;
  private String lastCluster = "none";
  private boolean manual = false;

  @Override
  public void init() {
    turret = new Turret(hardwareMap);       // zeroes: must point straight ahead
    turret.setTargetAngleRad(0);
    tracker = new HiveTracker(hardwareMap); // starts the camera
    telemetry.addLine("Turret zeroed (must be straight ahead). Hold LB to auto-aim.");
    telemetry.update();
  }

  @Override
  public void loop() {
    double stick = gamepad1.right_stick_x;
    boolean aiming = gamepad1.left_bumper;

    // ---- Find the best HIVE cluster in the newest detections ----
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

    String mode;
    if (Math.abs(stick) > STICK_DEADBAND && !aiming) {
      // ---- Manual ----
      manual = true;
      double angle = turret.getAngleRad();
      double power = -stick * MANUAL_POWER;   // stick left -> turret left
      if (angle >= RobotConstants.Turret.MAX_ANGLE_RAD && power > 0) power = 0;
      if (angle <= RobotConstants.Turret.MIN_ANGLE_RAD && power < 0) power = 0;
      turret.setRawPower(power);
      mode = "MANUAL";
    } else {
      if (manual) {
        // Just left manual: hold wherever the turret is now.
        turret.setTargetAngleRad(turret.getAngleRad());
        manual = false;
      }

      if (aiming && newFrame) {
        // Bearing + = target to the LEFT; turret + = LEFT. Correct by a fraction of the bearing.
        double target = turret.getAngleRad() + AIM_GAIN * Math.toRadians(best.ftcPose.bearing);
        turret.setTargetAngleRad(target);   // clipped to the software limits inside Turret
        lastFrameNano = best.frameAcquisitionNanoTime;
      }

      turret.update(0);   // robot not rotating during this test

      if (!aiming) mode = "HOLD";
      else if (!seenRecently) mode = "AIM: NO TARGET (holding)";
      else if (Math.abs(lastBearingDeg) <= AIMED_DEG) mode = "AIM: ON TARGET";
      else mode = "AIM: tracking";
    }

    telemetry.addData("Mode", mode);
    telemetry.addData("Cluster", seenRecently ? lastCluster : "none");
    telemetry.addData("Bearing to target (deg)", seenRecently ? String.format("%+.1f", lastBearingDeg) : "-");
    telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDeg());
    telemetry.addData("Turret error (deg)", "%+.1f", turret.getErrorDeg());
    telemetry.update();
  }

  @Override
  public void stop() {
    turret.stop();
    tracker.close();
  }
}