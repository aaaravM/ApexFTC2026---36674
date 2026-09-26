package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BallSorter;
import org.firstinspires.ftc.teamcode.util.BallType;

/**
 * STEP 6. Finds the hue windows that separate POLLEN from RED and BLUE NECTAR on your sensor, in
 * your lighting.
 *
 * <h2>Procedure</h2>
 * Hold each ball at the sensor in turn and press the matching button to record the hue range it
 * sweeps through. Roll the ball a little while it is recording so you catch the variation across its
 * surface -- the printed seams and any scuffs shift the hue a few degrees, and you want the window
 * wide enough to cover that.
 *
 * <ul>
 *   <li><b>A</b> while holding POLLEN (yellow)</li>
 *   <li><b>B</b> while holding RED NECTAR</li>
 *   <li><b>X</b> while holding BLUE NECTAR</li>
 *   <li><b>Y</b> to clear</li>
 * </ul>
 *
 * Then widen each recorded range by about 5 degrees on each side and put them into
 * {@code RobotConstants.Colors}. Check the three windows do not overlap; if they do, the sensor is
 * too far from the ball or there is too much ambient light reaching it.
 *
 * <h2>Also record two things that are easy to forget</h2>
 * <ol>
 *   <li><b>The empty reading.</b> Take the ball away and look at the saturation. Set
 *       {@code MIN_SATURATION} comfortably above it, or an empty hopper will classify as whatever
 *       colour the nearest wall is.</li>
 *   <li><b>The distance with a ball present versus absent.</b> Set {@code BALL_PRESENT_MM} between
 *       them. This is the more reliable of the two presence checks, so use it if your sensor has
 *       ranging.</li>
 * </ol>
 */
@TeleOp(name = "T6 Color Sensor Calibration", group = "Tuning")
public class T6_ColorSensorCalibration extends LinearOpMode {

    private static final class Range {
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        double minSat = Double.MAX_VALUE, maxSat = -Double.MAX_VALUE;
        int samples;
        void add(double hue, double sat) {
            min = Math.min(min, hue);
            max = Math.max(max, hue);
            minSat = Math.min(minSat, sat);
            maxSat = Math.max(maxSat, sat);
            samples++;
        }
        void clear() {
            min = Double.MAX_VALUE; max = -Double.MAX_VALUE;
            minSat = Double.MAX_VALUE; maxSat = -Double.MAX_VALUE;
            samples = 0;
        }
        String text() {
            if (samples == 0) return "not recorded";
            return String.format("hue %5.1f to %5.1f   sat %.2f to %.2f   (%d samples)",
                    min, max, minSat, maxSat, samples);
        }
    }

    @Override
    public void runOpMode() {
        BallSorter sorter = new BallSorter(hardwareMap);

        Range pollen = new Range();
        Range red = new Range();
        Range blue = new Range();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Press START, then hold each ball at the sensor.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            sorter.update();

            double hue = sorter.getHue();
            double sat = sorter.getSaturation();

            if (gamepad1.a) pollen.add(hue, sat);
            if (gamepad1.b) red.add(hue, sat);
            if (gamepad1.x) blue.add(hue, sat);
            if (gamepad1.y) { pollen.clear(); red.clear(); blue.clear(); }

            telemetry.addLine("=== LIVE ===");
            telemetry.addData("Hue", "%6.1f deg", hue);
            telemetry.addData("Saturation", "%.3f", sat);
            telemetry.addData("Value", "%.3f", sorter.getValue());
            if (sorter.hasDistanceSensor()) {
                telemetry.addData("Distance", "%.1f mm", sorter.getDistanceMm());
            } else {
                telemetry.addLine("Distance: sensor has no ranging (not a Color Sensor V3?)");
            }
            telemetry.addData("Ball present", sorter.isBallPresent());
            telemetry.addData("Classified as", sorter.getBallType());
            if (sorter.getBallType() != BallType.NONE) {
                telemetry.addData("Scorable by us", !sorter.shouldReject());
            }

            telemetry.addLine();
            telemetry.addLine("=== RECORDED (hold the button while rolling the ball) ===");
            telemetry.addData("A  POLLEN     ", pollen.text());
            telemetry.addData("B  NECTAR RED ", red.text());
            telemetry.addData("X  NECTAR BLUE", blue.text());
            telemetry.addLine("Y = clear all");

            telemetry.addLine();
            telemetry.addLine("Widen each range ~5 deg per side, then put into");
            telemetry.addLine("RobotConstants.Colors. Check they do not overlap.");
            telemetry.update();
        }
    }
}
