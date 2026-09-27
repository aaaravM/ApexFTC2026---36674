package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Flywheel shooter, controlled by RPM rather than by power.
 *
 * <p>Hardware here is one goBILDA 5203-2402-0001: bare 1:1, 6000 RPM free speed, 28 counts/rev,
 * 1.47 kg.cm stall, 9.2 A stall current.
 *
 * <h2>Why closed-loop velocity and not just a power number</h2>
 * A power number gives a different RPM at 13.5 V than at 11.8 V, and your battery will be somewhere
 * in between by the third match. Worse, the flywheel slows on every shot as it dumps energy into the
 * ball, so shots two and three of a burst fly shorter than shot one. Both problems disappear when you
 * command an RPM, let the Hub's velocity PIDF hold it, and wait for {@link #isAtSpeed()} before
 * feeding a ball.
 *
 * <h2>This motor is a better choice than it looks</h2>
 * BIOBUZZ shots are a soft lob, not a rocket. A 14 ft shot at a 40 degree hood from a 14 in muzzle
 * needs only about 7.3 m/s of ball exit speed -- roughly <b>2900 RPM on a 4 in wheel</b> at 0.5
 * transfer efficiency. The whole game lives in a <b>1600-2900 RPM</b> band, which is about 48% of
 * this motor's free speed.
 *
 * <p>A brushed DC motor makes peak mechanical power at exactly 50% of free speed. So a bare 1:1 5203
 * held around 2700-2900 RPM sits essentially on its peak-power point (22.6 W of the 22.64 W
 * available). <b>Do not belt it.</b> Any gearing, up or down, moves the operating point off peak
 * power and makes recovery between shots worse, not better.
 *
 * <p>One motor is also plenty. Per-shot energy is 1.1 J for POLLEN and 1.8 J for NECTAR against
 * roughly 25 J stored in a 280 g wheel, so droop is 65-108 RPM and recovery is 0.05-0.08 s -- well
 * inside any realistic firing cadence. A second motor buys nothing here.
 *
 * <h2>Two things the software cannot fix</h2>
 * <ol>
 *   <li><b>The ball size gap.</b> POLLEN is 2.8 in and NECTAR is 3.6 in. That 0.8 in is 2-4x the
 *       usable compression range of a compliant wheel, so one fixed gap cannot grip both -- a gap
 *       set for NECTAR will barely touch POLLEN. This needs a sprung or floating hood. No PIDF value
 *       compensates for a ball that is not touching the wheel.</li>
 *   <li><b>Velocity quantisation.</b> 28 counts/rev is the coarsest feedback on the robot. This class
 *       averages {@link RobotConstants.Shoot#RPM_SAMPLE_COUNT} samples to claw some of it back, which
 *       is what makes a 100 RPM tolerance usable at all.</li>
 * </ol>
 */
public class Shooter {

    private final DcMotorEx primary;
    private final DcMotorEx secondary;   // null when USE_DUAL_FLYWHEEL is false

    /** What the ballistics asked for. */
    private double goalRpm;
    /** What we are actually commanding right now, ramped toward {@link #goalRpm}. */
    private double commandedRpm;

    private final double[] rpmSamples = new double[Math.max(1, RobotConstants.Shoot.RPM_SAMPLE_COUNT)];
    private int sampleIndex;
    private boolean samplesFilled;

    private double atSpeedSinceSec = -1;
    private long lastCommandNano;

    public Shooter(HardwareMap hardwareMap) {
        primary = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
        configure(primary);

        if (RobotConstants.Shoot.USE_DUAL_FLYWHEEL) {
            secondary = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL_2);
            configure(secondary);
            // Opposed flywheels must spin opposite ways to throw the ball forward.
            secondary.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            secondary = null;
        }

        lastCommandNano = System.nanoTime();
    }

    private void configure(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast, not brake. Braking a flywheel wastes energy and stresses the gearbox.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyPidf(motor);
    }

    private void applyPidf(DcMotorEx motor) {
        motor.setVelocityPIDFCoefficients(
                RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
                RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);
    }

    /**
     * Command a flywheel speed in RPM.
     *
     * <p>Call this every loop. The commanded value is <b>ramped</b> toward the goal at
     * {@link RobotConstants.Shoot#RPM_RAMP_PER_SEC} rather than stepped, because a step draws close
     * to the motor's 9.2 A stall current for over a second -- and doing that while four drive motors
     * are accelerating is how you brown out a Control Hub mid-match.
     */
    public void setTargetRpm(double rpm) {
        goalRpm = Range.clip(rpm, 0, RobotConstants.Shoot.MAX_FLYWHEEL_RPM);

        long now = System.nanoTime();
        double dt = (now - lastCommandNano) / 1e9;
        lastCommandNano = now;
        if (dt <= 0 || dt > 0.25) dt = 0.02;   // guard a stalled or first loop

        if (RobotConstants.Shoot.RPM_RAMP_PER_SEC <= 0) {
            commandedRpm = goalRpm;
        } else {
            double maxStep = RobotConstants.Shoot.RPM_RAMP_PER_SEC * dt;
            double error = goalRpm - commandedRpm;
            commandedRpm += Range.clip(error, -maxStep, maxStep);
        }

        double ticksPerSecond = commandedRpm / 60.0 * RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV;
        primary.setVelocity(ticksPerSecond);
        if (secondary != null) secondary.setVelocity(ticksPerSecond);
    }

    /**
     * Let the flywheel coast down. Prefer this to {@link #stop()} between shots.
     *
     * <p>{@code setVelocity(0)} in RUN_USING_ENCODER does <b>not</b> coast -- the Hub actively brakes
     * the wheel to a stop, regardless of {@code setZeroPowerBehavior(FLOAT)}. That throws away all
     * the stored energy and then pays to rebuild it. Idle current is only 0.25 A, so leaving the
     * wheel spinning between shots is nearly free and removes the spin-up wait entirely.
     */
    public void idle() {
        goalRpm = 0;
        commandedRpm = 0;
        primary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        primary.setPower(0);
        if (secondary != null) {
            secondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            secondary.setPower(0);
        }
        resetSamples();
    }

    /** Actively brake to a stop. Use at OpMode teardown; use {@link #idle()} during a match. */
    public void stop() {
        goalRpm = 0;
        commandedRpm = 0;
        primary.setVelocity(0);
        if (secondary != null) secondary.setVelocity(0);
        resetSamples();
    }

    /** The RPM the ballistics asked for. */
    public double getTargetRpm() { return goalRpm; }

    /** The RPM currently being commanded, which trails the goal while ramping. */
    public double getCommandedRpm() { return commandedRpm; }

    /**
     * Measured flywheel speed, RPM, averaged over the last
     * {@link RobotConstants.Shoot#RPM_SAMPLE_COUNT} reads to fight 28-counts/rev quantisation.
     *
     * <p>Each call takes a new sample, so call it once per loop.
     */
    public double getCurrentRpm() {
        double instant = readInstantRpm(primary);
        if (secondary != null) {
            // Average the pair. Ignoring the second motor would hide a dead or mis-wired one.
            instant = (instant + readInstantRpm(secondary)) / 2.0;
        }

        rpmSamples[sampleIndex] = instant;
        sampleIndex = (sampleIndex + 1) % rpmSamples.length;
        if (sampleIndex == 0) samplesFilled = true;

        int count = samplesFilled ? rpmSamples.length : Math.max(1, sampleIndex);
        double sum = 0;
        for (int i = 0; i < count; i++) sum += rpmSamples[i];
        return sum / count;
    }

    /** One unaveraged read, in RPM. */
    public double getInstantRpm() { return readInstantRpm(primary); }

    private static double readInstantRpm(DcMotorEx motor) {
        // getVelocity() with no unit returns RAW encoder ticks/sec. Never use the AngleUnit
        // overloads on a 5203: they scale by the configured MotorConfigurationType ratio, and the
        // SDK has no 5203 entry -- the nearest one hardcodes 99.5:1, which is wrong for every one
        // of this robot's motors.
        return motor.getVelocity() / RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public double getRpmError() { return goalRpm - getCurrentRpm(); }

    /**
     * True once the wheel has been within tolerance continuously for
     * {@link RobotConstants.Shoot#READY_STABLE_SEC}, and the ramp has reached the goal.
     *
     * <p>The "continuously" part matters. A wheel spinning up passes through its target on the way
     * to overshooting, and a ball fed at that instant gets a speed that is right for one millisecond
     * and wrong for the rest of the contact. Requiring it to settle costs about a tenth of a second
     * per shot and buys real consistency -- and with a HIVE TIP worth ten times an in-CELL element,
     * consistency is what scores.
     */
    public boolean isAtSpeed() {
        if (goalRpm <= 0) return false;
        // Still ramping: the commanded value has not even reached the goal yet.
        if (Math.abs(goalRpm - commandedRpm) > RobotConstants.Shoot.RPM_TOLERANCE) {
            atSpeedSinceSec = -1;
            return false;
        }

        boolean within = Math.abs(getRpmError()) <= RobotConstants.Shoot.RPM_TOLERANCE;
        double now = System.nanoTime() / 1e9;
        if (!within) {
            atSpeedSinceSec = -1;
            return false;
        }
        if (atSpeedSinceSec < 0) atSpeedSinceSec = now;
        return (now - atSpeedSinceSec) >= RobotConstants.Shoot.READY_STABLE_SEC;
    }

    /** Raw power, for the tuning OpMode only -- bypasses velocity control. */
    public void setRawPower(double power) {
        primary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        primary.setPower(power);
        if (secondary != null) {
            secondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            secondary.setPower(power);
        }
    }

    /**
     * Return to closed-loop velocity control after {@link #setRawPower(double)} or {@link #idle()}.
     *
     * <p>Re-applies the PIDF coefficients as well as the run mode. Switching a motor out of
     * RUN_USING_ENCODER and back can leave it on the Hub's defaults rather than ours, and a flywheel
     * silently running someone else's F term is a miserable thing to chase.
     */
    public void restoreVelocityMode() {
        primary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidf(primary);
        if (secondary != null) {
            secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyPidf(secondary);
        }
        resetSamples();
    }

    /**
     * Motor current in amps. Published stall current is 9.2 A; a reading near that means the wheel
     * is jammed or a ball is stuck against it.
     */
    public double getCurrentAmps() {
        return primary.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }

    private void resetSamples() {
        sampleIndex = 0;
        samplesFilled = false;
        for (int i = 0; i < rpmSamples.length; i++) rpmSamples[i] = 0;
        atSpeedSinceSec = -1;
    }
}
