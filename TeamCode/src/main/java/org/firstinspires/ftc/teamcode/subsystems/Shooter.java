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
 * <h2>Why closed-loop velocity and not just a power number</h2>
 * A power number gives a different RPM at 13.5 V than at 11.8 V, and your battery will be
 * somewhere in between by the third match of the day. Worse, the flywheel slows on every shot as
 * it dumps energy into the ball, so shots two and three of a burst go shorter than shot one. Both
 * problems disappear when you command an RPM and let the motor controller's velocity PIDF hold it,
 * and wait for {@link #isAtSpeed()} before feeding a ball.
 *
 * <h2>Tuning the PIDF</h2>
 * F does almost all the work for a flywheel; P cleans up the recovery after a shot.
 * A reasonable starting point for F is {@code 32767 / max_ticks_per_second}. Set P to about a
 * third of F, leave I at 0 (it causes overshoot on a high-inertia wheel), and leave D at 0.
 */
public class Shooter {

    private final DcMotorEx primary;
    private final DcMotorEx secondary;   // null if single-flywheel

    private double targetRpm;
    private double atSpeedSinceSec = -1;

    public Shooter(HardwareMap hardwareMap) {
        primary = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL);
        primary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        primary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast, not brake. Braking a flywheel wastes energy and stresses the gearbox.
        primary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        primary.setVelocityPIDFCoefficients(
                RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
                RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);

        if (RobotConstants.Shoot.USE_DUAL_FLYWHEEL) {
            secondary = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FLYWHEEL_2);
            secondary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            secondary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            secondary.setVelocityPIDFCoefficients(
                    RobotConstants.Shoot.FLYWHEEL_P, RobotConstants.Shoot.FLYWHEEL_I,
                    RobotConstants.Shoot.FLYWHEEL_D, RobotConstants.Shoot.FLYWHEEL_F);
            // Opposed flywheels must spin opposite ways to throw the ball forward.
            secondary.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            secondary = null;
        }
    }

    /** Command a flywheel speed in RPM. 0 stops it. */
    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0, RobotConstants.Shoot.MAX_FLYWHEEL_RPM);
        double ticksPerSecond = targetRpm / 60.0 * RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV;
        primary.setVelocity(ticksPerSecond);
        if (secondary != null) secondary.setVelocity(ticksPerSecond);
    }

    public void stop() { setTargetRpm(0); }

    public double getTargetRpm() { return targetRpm; }

    /** Measured flywheel speed, RPM. */
    public double getCurrentRpm() {
        return primary.getVelocity() / RobotConstants.Shoot.FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public double getRpmError() { return targetRpm - getCurrentRpm(); }

    /**
     * True once the wheel has been within tolerance continuously for
     * {@link RobotConstants.Shoot#READY_STABLE_SEC}.
     *
     * <p>The "continuously" part matters. A wheel spinning up passes through the target on its way
     * to overshooting, and a ball fed at that instant gets a speed that is correct for one
     * millisecond and wrong for the rest of the contact. Requiring the wheel to have settled costs
     * about a tenth of a second per shot and buys real consistency.
     */
    public boolean isAtSpeed() {
        if (targetRpm <= 0) return false;
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

    /** Return to closed-loop velocity control after {@link #setRawPower(double)}. */
    public void restoreVelocityMode() {
        primary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (secondary != null) secondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
