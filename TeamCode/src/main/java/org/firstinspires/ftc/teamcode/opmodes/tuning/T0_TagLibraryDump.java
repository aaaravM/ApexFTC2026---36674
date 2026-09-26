package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;
import java.util.List;

/**
 * STEP 0. Prints the entire BIOBUZZ AprilTag library that ships inside SDK 12.0 -- every tag ID,
 * every tag size, and every cluster name.
 *
 * <p><b>Needs no hardware whatsoever.</b> No motors, no camera, no field, no odometry. A Control Hub
 * (or even just a phone as Robot Controller) and a Driver Station is enough. Run it on day one.
 *
 * <h2>Why bother</h2>
 * The BIOBUZZ tag IDs and cluster names are not published anywhere convenient -- they live inside the
 * SDK's compiled tag library. But your code needs one of them:
 * {@code RobotConstants.Hive.TARGET_CLUSTER_SHORT_NAME}, so the turret locks onto <i>your</i>
 * alliance's HIVE and not the opponent's. This OpMode reads the definitive list straight from the
 * SDK, which beats guessing and beats waiting until you have a field in front of you.
 *
 * <p>Write down what it prints. Those strings are the contract between your code and the field.
 *
 * <h2>Reading the output</h2>
 * <ul>
 *   <li><b>Clusters</b> are what a BIOBUZZ CELL reports as. The {@code shortName} is the string to
 *       copy into the constants file.</li>
 *   <li><b>Tags</b> are the individual members. You will not normally handle these directly, because
 *       a CELL returns one cluster detection rather than four singletons.</li>
 *   <li><b>Field position</b> will most likely be absent or meaningless for BIOBUZZ tags. That is
 *       expected and correct: the HIVE tips during the match, so the tags move, and the SDK release
 *       notes say plainly that they are not suitable for absolute field localisation. If you see
 *       positions here, do not build a localiser on them.</li>
 * </ul>
 *
 * <p>Use the D-pad to scroll -- the list is longer than one telemetry screen.
 */
@TeleOp(name = "T0 Tag Library Dump (no hardware)", group = "Tuning")
public class T0_TagLibraryDump extends LinearOpMode {

    private static final int LINES_PER_PAGE = 12;

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);

        // Built into SDK 12.0 alongside getCurrentGameTagLibrary(). Asking for BioBuzz explicitly
        // guards against a future SDK where "current" has moved on to the next game.
        AprilTagLibrary bioBuzz = AprilTagGameDatabase.getBioBuzzTagLibrary();

        // Note: these loops iterate the getter directly rather than assigning it to a local. A
        // for-each over a method call compiles whether the SDK returns an array or a List, so this
        // keeps working if FIRST ever changes the collection type.
        List<String> clusterLines = new ArrayList<>();
        int clusterCount = 0;
        for (AprilTagClusterMetadata cluster : bioBuzz.getAllClusters()) {
            clusterCount++;
            clusterLines.add(String.format("shortName=\"%s\"", cluster.shortName));
            clusterLines.add(String.format("   name: %s", cluster.name));
            if (cluster.fieldPosition != null) {
                clusterLines.add(String.format("   pos: %s (%s)",
                        cluster.fieldPosition, cluster.distanceUnit));
            } else {
                clusterLines.add("   pos: none (tag moves -- expected)");
            }
        }

        List<String> tagLines = new ArrayList<>();
        int tagCount = 0;
        for (AprilTagMetadata tag : bioBuzz.getAllTags()) {
            tagCount++;
            tagLines.add(String.format("id=%-4d size=%.2f %s  %s",
                    tag.id, tag.tagsize, tag.distanceUnit, tag.name));
        }

        List<String> lines = new ArrayList<>();
        lines.add("=== CLUSTERS (" + clusterCount + ") ===");
        lines.add("copy a shortName into RobotConstants.Hive");
        lines.addAll(clusterLines);
        lines.add("");
        lines.add("=== INDIVIDUAL TAGS (" + tagCount + ") ===");
        lines.addAll(tagLines);

        telemetry.addLine("Press START to browse the tag library.");
        telemetry.addData("clusters", clusterCount);
        telemetry.addData("tags", tagCount);
        telemetry.update();
        waitForStart();

        int scroll = 0;
        boolean upWas = false, downWas = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_down && !downWas) {
                scroll = Math.min(scroll + LINES_PER_PAGE, Math.max(0, lines.size() - 1));
            }
            downWas = gamepad1.dpad_down;

            if (gamepad1.dpad_up && !upWas) scroll = Math.max(0, scroll - LINES_PER_PAGE);
            upWas = gamepad1.dpad_up;

            telemetry.addData("line", "%d-%d of %d   (dpad up/down)",
                    scroll + 1, Math.min(scroll + LINES_PER_PAGE, lines.size()), lines.size());
            telemetry.addLine();
            for (int i = scroll; i < Math.min(scroll + LINES_PER_PAGE, lines.size()); i++) {
                telemetry.addLine(lines.get(i));
            }
            telemetry.update();
        }
    }
}
