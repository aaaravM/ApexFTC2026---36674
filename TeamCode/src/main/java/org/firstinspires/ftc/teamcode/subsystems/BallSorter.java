package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.BallType;

/**
 * Identifies which ball is sitting in the hopper throat, from a colour sensor.
 *
 * <p>Mount the sensor looking at the ball just before the feeder, 5-15 mm away. A REV Color Sensor
 * V3 is ideal because it also gives a distance reading, which tells you a ball is actually there
 * rather than that you are reading the far wall of the hopper.
 *
 * <h2>Classify on hue, not on raw RGB</h2>
 * Raw red/green/blue channels all scale with how much light is hitting the ball, so a yellow ball
 * in shadow and a red ball in bright light can produce similar numbers. Hue is the angle around the
 * colour wheel and is largely independent of brightness, so a single hue window per colour holds up
 * across venues far better. Saturation then acts as the "is this actually a coloured object"
 * confidence check.
 *
 * <h2>Two jobs, not one</h2>
 * <ol>
 *   <li><b>Shooter power.</b> NECTAR needs more RPM than POLLEN for the same distance -- see
 *       {@link BallType#transferEfficiency}.</li>
 *   <li><b>Rejection.</b> Only our own alliance's NECTAR scores for us. Feeding the opponent's
 *       NECTAR into our HIVE spends a cycle for nothing. {@link #shouldReject()} catches it.</li>
 * </ol>
 */
public class BallSorter {

    private final NormalizedColorSensor sensor;
    private final DistanceSensor distance;   // null if the configured sensor has no ranging

    private final float[] hsv = new float[3];
    private BallType lastType = BallType.NONE;
    private double lastHue, lastSaturation, lastValue, lastDistanceMm = Double.NaN;

    public BallSorter(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, RobotConstants.Names.BALL_COLOR);
        sensor.setGain(RobotConstants.Colors.SENSOR_GAIN);
        distance = (sensor instanceof DistanceSensor) ? (DistanceSensor) sensor : null;
    }

    /** Read the sensor and classify. Call once per loop. */
    public void update() {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        lastHue = hsv[0];
        lastSaturation = hsv[1];
        lastValue = hsv[2];

        if (distance != null) {
            lastDistanceMm = distance.getDistance(DistanceUnit.MM);
        }

        lastType = classify();
    }

    private BallType classify() {
        // No ball in front of the sensor at all.
        if (!isBallPresent()) return BallType.NONE;

        // Washed-out or grey reading: do not guess.
        if (lastSaturation < RobotConstants.Colors.MIN_SATURATION) return BallType.NONE;

        if (inWindow(lastHue, RobotConstants.Colors.POLLEN_HUE_MIN,
                              RobotConstants.Colors.POLLEN_HUE_MAX)) {
            return BallType.POLLEN;
        }
        if (inWindow(lastHue, RobotConstants.Colors.RED_HUE_MIN,
                              RobotConstants.Colors.RED_HUE_MAX)) {
            return BallType.NECTAR_RED;
        }
        if (inWindow(lastHue, RobotConstants.Colors.BLUE_HUE_MIN,
                              RobotConstants.Colors.BLUE_HUE_MAX)) {
            return BallType.NECTAR_BLUE;
        }
        return BallType.NONE;
    }

    /**
     * Hue window test that handles wrapping through 0. Red spans roughly 340-360 and 0-20, so a
     * naive {@code min <= h && h <= max} would never match it.
     */
    private static boolean inWindow(double hue, double min, double max) {
        if (min <= max) return hue >= min && hue <= max;
        return hue >= min || hue <= max;     // the window wraps past 360
    }

    /**
     * Is there a ball at the sensor? Uses the distance reading when available, and otherwise falls
     * back to "the colour is saturated enough that something coloured is in front of us".
     */
    public boolean isBallPresent() {
        if (distance != null) {
            if (Double.isNaN(lastDistanceMm)) return false;
            return lastDistanceMm <= RobotConstants.Colors.BALL_PRESENT_MM;
        }
        return lastSaturation >= RobotConstants.Colors.MIN_SATURATION;
    }

    /** The ball we think is loaded. {@link BallType#NONE} if empty or unrecognised. */
    public BallType getBallType() { return lastType; }

    /**
     * True if this ball cannot score for us and should be spat back out. POLLEN is always fine;
     * the opponent's NECTAR is not.
     */
    public boolean shouldReject() {
        return lastType != BallType.NONE && !lastType.isScorableBy(RobotConstants.alliance);
    }

    // Raw values, for the calibration OpMode and for telemetry while debugging.
    public double getHue() { return lastHue; }
    public double getSaturation() { return lastSaturation; }
    public double getValue() { return lastValue; }
    public double getDistanceMm() { return lastDistanceMm; }
    public boolean hasDistanceSensor() { return distance != null; }
}
