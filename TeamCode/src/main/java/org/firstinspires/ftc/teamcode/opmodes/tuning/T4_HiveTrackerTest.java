package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.util.Ballistics;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * STEP 4. Proves the vision-plus-odometry chain before any motor is involved. Push the robot around
 * by hand while pointing the camera at a HIVE.
 *
 * <h2>The thing this OpMode exists to tell you</h2>
 * <b>The BIOBUZZ CELL cluster names.</b> They are baked into the SDK's tag library, not published in
 * a form we could look up, so read them off your own field here and then put your alliance's name
 * into {@code RobotConstants.Hive.TARGET_CLUSTER_SHORT_NAME}. Until you do, the tracker accepts any
 * cluster it sees -- which in a real match means it will happily aim at the opponent's HIVE.
 *
 * <h2>What good looks like</h2>
 * <ul>
 *   <li><b>Cluster detected, not singletons.</b> A BIOBUZZ CELL reports as one cluster when at
 *       least one of its four member tags is visible; the members do not also appear individually.
 *       If you only ever see singletons, you are looking at something that is not a CELL.</li>
 *   <li><b>Field X/Y holds still while you move.</b> This is the real test. The CELL is not moving,
 *       so its estimated FIELD position must stay put as you walk the robot around. If it slides as
 *       you drive, the camera mount offsets or the camera yaw in {@code RobotConstants.Camera} are
 *       wrong. If it jumps when you rotate fast, the latency compensation needs a look.</li>
 *   <li><b>The estimate survives occlusion.</b> Cover the camera. "Live" goes false, age climbs,
 *       but the field estimate stays and the ballistic solution keeps updating from odometry.
 *       That is odometry bridging a vision dropout, which is the whole point of the design.</li>
 * </ul>
 */
@TeleOp(name = "T4 Hive Tracker Test", group = "Tuning")
public class T4_HiveTrackerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);
        HiveTracker hive = new HiveTracker(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Waiting for the camera...");
        telemetry.update();

        hive.setManualExposure();

        telemetry.addLine("Camera ready. Press START.");
        telemetry.addLine("Then point it at a HIVE and push the robot around.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            hive.update(odometry, 0);   // 0 = no turret on this test

            if (gamepad1.a) odometry.resetPoseAndCalibrateImu();
            if (gamepad1.b) hive.resetEstimate();

            telemetry.addLine("=== RAW DETECTIONS ===");
            telemetry.addData("count", hive.getDetectionCount());
            for (AprilTagDetection detection : hive.getRawDetections()) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                    telemetry.addLine(String.format("  single id=%d  %s", single.id,
                            single.metadata == null ? "(not in library)" : single.metadata.name));
                } else {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    // >>> THIS is the string to copy into TARGET_CLUSTER_SHORT_NAME <<<
                    telemetry.addLine(String.format("  CLUSTER shortName=\"%s\"  found=%d%%",
                            cluster.metadata == null ? "?" : cluster.metadata.shortName,
                            cluster.percentClusterFound));
                    if (cluster.metadata != null) {
                        telemetry.addLine(String.format("     full name: %s", cluster.metadata.name));
                    }
                }
            }

            telemetry.addLine();
            telemetry.addLine("=== ACCEPTED SIGHTING ===");
            telemetry.addData("cluster", hive.getLastClusterName());
            telemetry.addData("tags found", "%d%%", hive.getLastPercentFound());
            telemetry.addData("range/bearing/elev", "%6.1f in  %+6.1f deg  %+6.1f deg",
                    hive.getLastRange(), hive.getLastBearingDeg(), hive.getLastElevationDeg());

            telemetry.addLine();
            telemetry.addLine("=== FIELD ESTIMATE (should NOT move as you drive) ===");
            telemetry.addData("target X/Y/Z", "%+7.1f  %+7.1f  %+7.1f in",
                    hive.getTargetFieldX(), hive.getTargetFieldY(), hive.getTargetFieldZ());
            telemetry.addData("live?", hive.isSightingLive() ? "YES" : "no (dead reckoning)");
            telemetry.addData("age", "%.2f s", hive.getEstimateAgeSec());
            telemetry.addData("usable?", hive.hasTarget() ? "YES" : "NO -- too stale");

            telemetry.addLine();
            telemetry.addLine("=== ROBOT ===");
            telemetry.addData("pose", "X %+7.1f  Y %+7.1f  H %+6.1f deg",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
            telemetry.addData("field vel", "%+6.1f  %+6.1f in/s",
                    odometry.getFieldVelX(), odometry.getFieldVelY());

            if (hive.hasTarget()) {
                Ballistics.Solution solution = Ballistics.solve(
                        odometry.getX(), odometry.getY(),
                        org.firstinspires.ftc.teamcode.RobotConstants.Shoot.MUZZLE_Z_IN,
                        hive.getTargetFieldX(), hive.getTargetFieldY(), hive.getTargetFieldZ(),
                        odometry.getFieldVelX(), odometry.getFieldVelY(),
                        odometry.getHeadingRad(), BallType.POLLEN);
                telemetry.addLine();
                telemetry.addLine("=== BALLISTIC SOLUTION (POLLEN) ===");
                telemetry.addLine("  " + solution);
            }

            telemetry.addLine();
            telemetry.addLine("A = reset odometry   B = clear target estimate");
            telemetry.update();
        }

        hive.close();
    }
}
