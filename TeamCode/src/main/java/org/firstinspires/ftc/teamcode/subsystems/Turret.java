package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Ballistics;

/**
 * The turret yaw axis. Holds a commanded angle in the ROBOT frame, or holds an angle in the FIELD
 * frame while the chassis moves underneath it.
 *
 * <h2>The rotation feedforward, and why it is the important part</h2>
 * A plain position PID will always lag when the chassis spins, because the target angle in the
 * robot frame is changing as fast as the chassis turns. The PID only reacts <i>after</i> an error
 * appears. The fix is a feedforward term: if the chassis is rotating counter-clockwise at
 * {@code w} rad/s, the turret must rotate clockwise at {@code w} rad/s just to stand still in the
 * field, so we add {@code -w * KV} to the output before the PID ever sees an error.
 *
 * <p>The Pinpoint gives us {@code w} directly from its fused IMU at 1500 Hz, which is far better
 * than differentiating a heading in our own 50 Hz loop. This is a concrete example of odometry
 * doing something for the turret that vision cannot.
 */
public class Turret {

    private final DcMotorEx motor;

    private double targetAngleRad;
    private double lastError;
    private double integral;
    private long lastNano;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.TURRET);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lastNano = System.nanoTime();
    }

    /**
     * Zero the encoder here. Call this with the turret physically centred and pointing straight
     * forward, during init. If you have a limit switch or hard stop, drive to it first.
     */
    public void zeroHere() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        integral = 0;
        lastError = 0;
    }

    /** Current turret angle in the ROBOT frame, radians. 0 = straight forward. */
    public double getAngleRad() {
        return motor.getCurrentPosition() / RobotConstants.Turret.TICKS_PER_RAD;
    }

    public double getAngleDeg() { return Math.toDegrees(getAngleRad()); }

    /** Command an angle in the ROBOT frame, radians, clamped to the mechanical limits. */
    public void setTargetAngleRad(double angleRad) {
        targetAngleRad = Range.clip(angleRad,
                RobotConstants.Turret.MIN_ANGLE_RAD, RobotConstants.Turret.MAX_ANGLE_RAD);
    }

    /**
     * Command an angle in the FIELD frame. The turret then holds that field bearing while the
     * chassis drives and spins around it.
     *
     * @param fieldAngleRad   where to point, radians CCW from field +X
     * @param robotHeadingRad current heading from odometry
     */
    public void setTargetFieldAngleRad(double fieldAngleRad, double robotHeadingRad) {
        setTargetAngleRad(Ballistics.wrapRad(fieldAngleRad - robotHeadingRad));
    }

    /**
     * Run one control iteration. Call once per loop, after {@link Odometry#update()}.
     *
     * @param headingVelRad chassis rotation rate from {@link Odometry#getHeadingVelRad()}
     */
    public void update(double headingVelRad) {
        long now = System.nanoTime();
        double dt = (now - lastNano) / 1e9;
        lastNano = now;
        if (dt <= 0 || dt > 0.25) dt = 0.02;   // guard against a stalled or first loop

        double error = targetAngleRad - getAngleRad();

        // Integral, with clamping so it cannot wind up while the turret is against a hard stop.
        integral = Range.clip(integral + error * dt, -0.5, 0.5);
        if (RobotConstants.Turret.KI == 0) integral = 0;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = RobotConstants.Turret.KP * error
                      + RobotConstants.Turret.KI * integral
                      + RobotConstants.Turret.KD * derivative;

        // Cancel chassis rotation before it becomes an error.
        output += -headingVelRad * RobotConstants.Turret.KV;

        // Break static friction, but only when we actually want to move.
        if (Math.abs(error) > RobotConstants.Turret.TOLERANCE_RAD) {
            output += Math.copySign(RobotConstants.Turret.KS, output);
        }

        // Refuse to push past the mechanical limits even if the PID asks.
        double angle = getAngleRad();
        if (angle <= RobotConstants.Turret.MIN_ANGLE_RAD && output < 0) output = 0;
        if (angle >= RobotConstants.Turret.MAX_ANGLE_RAD && output > 0) output = 0;

        motor.setPower(Range.clip(output,
                -RobotConstants.Turret.MAX_POWER, RobotConstants.Turret.MAX_POWER));
    }

    public boolean isOnTarget() {
        return Math.abs(targetAngleRad - getAngleRad()) < RobotConstants.Turret.TOLERANCE_RAD;
    }

    public double getErrorDeg() { return Math.toDegrees(targetAngleRad - getAngleRad()); }

    public double getTargetAngleDeg() { return Math.toDegrees(targetAngleRad); }

    public void stop() { motor.setPower(0); }

    /** Direct power, for the tuning OpMode only. */
    public void setRawPower(double power) {
        motor.setPower(Range.clip(power, -1, 1));
    }

    public int getEncoderTicks() { return motor.getCurrentPosition(); }
}
