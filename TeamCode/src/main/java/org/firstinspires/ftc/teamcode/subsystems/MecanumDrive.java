package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Ballistics;

/**
 * Mecanum drivetrain, with robot-centric and field-centric modes.
 *
 * <p>The wheel mixing matches the SDK's own samples and the Pinpoint's convention:
 * <b>+x forward, +y strafe left, +yaw counter-clockwise</b>. Keeping one convention across the
 * whole codebase is the single cheapest way to avoid sign bugs.
 */
public class MecanumDrive {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.BACK_RIGHT);

        frontLeft.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_LEFT));
        frontRight.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_RIGHT));
        backLeft.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_LEFT));
        backRight.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_RIGHT));

        // Brake when the stick is released. On a shooting robot this matters more than usual:
        // coasting to a stop means the odometry velocity is still non-zero when the driver thinks
        // the robot has stopped, so the turret keeps applying a lead it no longer needs.
        for (DcMotorEx m : all()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private DcMotorEx[] all() { return new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}; }

    private static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    /**
     * Robot-centric drive. Forward is whichever way the robot is facing.
     *
     * @param x   forward, -1..1
     * @param y   strafe left, -1..1
     * @param yaw counter-clockwise, -1..1
     */
    public void driveRobotCentric(double x, double y, double yaw) {
        double frontLeftPower  = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower   = x + y - yaw;
        double backRightPower  = x - y + yaw;

        // Scale all four down together if any exceeds 1, so the direction of travel is preserved.
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                              Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Field-centric drive: pushing the stick away from the driver moves the robot away from the
     * driver, whatever way the robot happens to be facing.
     *
     * <p>This is close to mandatory for a turret robot. The whole point of a turret is that the
     * chassis no longer has to point at the goal, so the driver should not have to think about
     * which way the chassis points either.
     *
     * @param fieldX field-forward, -1..1
     * @param fieldY field-left, -1..1
     * @param yaw    counter-clockwise, -1..1
     * @param headingRad current heading from odometry
     */
    public void driveFieldCentric(double fieldX, double fieldY, double yaw, double headingRad) {
        // Rotate the requested field-frame motion into the robot frame.
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);
        double robotX = fieldX * cos - fieldY * sin;
        double robotY = fieldX * sin + fieldY * cos;
        driveRobotCentric(robotX, robotY, yaw);
    }

    /**
     * Field-centric drive with the heading held at a target by a P controller. Use it when you want
     * the chassis square to the field while the turret does the aiming, or to hold a heading that
     * keeps the camera pointed at the HIVE.
     */
    public void driveFieldCentricWithHeadingLock(double fieldX, double fieldY,
                                                 double targetHeadingRad,
                                                 double currentHeadingRad) {
        double error = Ballistics.wrapRad(targetHeadingRad - currentHeadingRad);
        double yaw = Range.clip(error * RobotConstants.Drive.HEADING_LOCK_KP,
                -RobotConstants.Drive.HEADING_LOCK_MAX, RobotConstants.Drive.HEADING_LOCK_MAX);
        driveFieldCentric(fieldX, fieldY, yaw, currentHeadingRad);
    }

    public void stop() { driveRobotCentric(0, 0, 0); }

    /** Applies the deadband and squares the input for finer control near centre. */
    public static double shapeStick(double raw) {
        if (Math.abs(raw) < RobotConstants.Drive.STICK_DEADBAND) return 0;
        return Math.copySign(raw * raw, raw);
    }
}
