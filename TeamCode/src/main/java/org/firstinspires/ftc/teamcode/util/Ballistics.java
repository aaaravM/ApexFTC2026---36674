package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Solves "what turret angle and flywheel RPM will put this ball in that CELL, given that we are
 * driving at the same time".
 *
 * <h2>The idea in one paragraph</h2>
 * When the ball leaves a moving robot it keeps the robot's velocity. So the ball's true launch
 * velocity is {@code muzzle velocity + robot velocity}. That means we cannot just aim at the
 * target; we must aim at a <b>virtual target</b> displaced by {@code -robotVelocity * flightTime}.
 * Shoot at the virtual target as if standing still, add the robot's velocity back in, and the ball
 * arrives at the real target. The catch is that the flight time depends on where the virtual target
 * is, and the virtual target depends on the flight time -- so we iterate. It converges in two or
 * three passes because the correction is small compared to the range.
 *
 * <h2>The derivation</h2>
 * Let {@code p0} be the muzzle, {@code pt} the target, {@code vr} the robot's field velocity,
 * {@code Vm} the muzzle velocity we are solving for, and {@code G} gravity.
 * <pre>
 *   pt = p0 + (Vm + vr)*t + (0,0,-G)*t^2/2
 * </pre>
 * Rearranged, with the virtual target {@code pv = pt - vr*t}:
 * <pre>
 *   horizontal:  Vm_h = (pv_h - p0_h) / t          so |Vm_h| = d/t, aimed at pv
 *   vertical:    Vm_z = h/t + G*t/2                where h = pt_z - p0_z
 * </pre>
 * The hood fixes the launch pitch {@code theta}, which ties those two together:
 * <pre>
 *   tan(theta) = Vm_z / |Vm_h| = (h + G*t^2/2) / d
 *   =>  t = sqrt( 2*(d*tan(theta) - h) / G )
 * </pre>
 * So for a fixed hood angle the flight time falls straight out of the geometry -- no search needed
 * within an iteration. Then {@code muzzleSpeed = (d/t) / cos(theta)} and the azimuth is simply the
 * bearing to the virtual target. That is the whole solver.
 *
 * <h2>What this deliberately does not model</h2>
 * Air drag and spin (Magnus). Both are real on a 25 g foam ball. We correct for them with a single
 * empirical term, {@link BallType#dragPerInch}, applied as a speed scale that grows with range.
 * That is crude but it is one number per ball type and you can tune it in an afternoon, whereas a
 * real drag model needs a coefficient you cannot measure without a wind tunnel.
 */
public final class Ballistics {

    private Ballistics() {}

    /** Gravity in inches per second squared (9.80665 m/s^2). */
    public static final double G_IN_S2 = 386.0886;

    /** The answer, or an explanation of why there isn't one. */
    public static final class Solution {
        /** True only if every check passed and it is safe to shoot. */
        public boolean valid;

        /** Azimuth to point the muzzle, FIELD frame, radians CCW from +X. */
        public double fieldAzimuthRad;

        /** Turret angle in the ROBOT frame, radians. This is what the turret PID wants. */
        public double turretAngleRad;

        /** Required speed of the ball as it leaves the muzzle, inches/sec. */
        public double muzzleSpeedIn;

        /** Flywheel speed that produces that muzzle speed for this ball type, RPM. */
        public double flywheelRpm;

        /** Predicted time of flight, seconds. */
        public double flightTimeSec;

        /** Horizontal distance to the VIRTUAL target (what we actually aim at), inches. */
        public double virtualDistanceIn;

        /** Horizontal distance to the REAL target, inches. Useful for telemetry comparison. */
        public double actualDistanceIn;

        /** How far the moving-shot correction moved our aim, radians. Zero when stationary. */
        public double leadAngleRad;

        /** Human-readable reason when {@link #valid} is false. */
        public String failReason = "";

        @Override public String toString() {
            if (!valid) return "INVALID: " + failReason;
            return String.format(
                    "az=%.1fdeg turret=%.1fdeg rpm=%.0f tof=%.2fs d=%.1fin lead=%.1fdeg",
                    Math.toDegrees(fieldAzimuthRad), Math.toDegrees(turretAngleRad),
                    flywheelRpm, flightTimeSec, actualDistanceIn, Math.toDegrees(leadAngleRad));
        }
    }

    /**
     * Full moving-shot solve.
     *
     * @param muzzleX      muzzle position, field frame, inches
     * @param muzzleY      muzzle position, field frame, inches
     * @param muzzleZ      muzzle height above the floor, inches
     * @param targetX      aim point, field frame, inches
     * @param targetY      aim point, field frame, inches
     * @param targetZ      aim point height above the floor, inches
     * @param robotVx      robot velocity, FIELD frame, inches/sec
     * @param robotVy      robot velocity, FIELD frame, inches/sec
     * @param robotHeading current robot heading, radians CCW
     * @param ball         which ball is about to be fired (drives the RPM conversion)
     * @return a {@link Solution}; always non-null, check {@link Solution#valid}
     */
    public static Solution solve(double muzzleX, double muzzleY, double muzzleZ,
                                 double targetX, double targetY, double targetZ,
                                 double robotVx, double robotVy,
                                 double robotHeading,
                                 BallType ball) {

        Solution s = new Solution();

        final double theta = Math.toRadians(RobotConstants.Shoot.LAUNCH_PITCH_DEG);
        final double tanTheta = Math.tan(theta);
        final double cosTheta = Math.cos(theta);
        final double h = targetZ - muzzleZ;     // how much higher the target is than the muzzle

        // Geometry to the real target, for telemetry and for the no-solution messages.
        s.actualDistanceIn = Math.hypot(targetX - muzzleX, targetY - muzzleY);
        final double actualAzimuth = Math.atan2(targetY - muzzleY, targetX - muzzleX);

        if (theta <= 0 || theta >= Math.PI / 2) {
            s.failReason = "LAUNCH_PITCH_DEG must be strictly between 0 and 90";
            return s;
        }

        // ---- Fixed-point iteration on flight time --------------------------------------------
        // Start with t = 0, i.e. "assume we are standing still", then refine.
        double t = 0;
        double d = s.actualDistanceIn;
        double azimuth = actualAzimuth;

        for (int i = 0; i < 6; i++) {
            // Where the target effectively is, given that the ball inherits our velocity.
            double vtx = targetX - robotVx * t;
            double vty = targetY - robotVy * t;

            double dx = vtx - muzzleX;
            double dy = vty - muzzleY;
            d = Math.hypot(dx, dy);
            azimuth = Math.atan2(dy, dx);

            // t^2 = 2*(d*tan(theta) - h)/G. If the bracket is non-positive the target sits at or
            // above the launch line, so no shot exists at this hood angle.
            double bracket = d * tanTheta - h;
            if (bracket <= 0) {
                s.failReason = String.format(
                        "target too high/close for a %.0f deg hood (need d*tan > rise; d=%.1f, rise=%.1f)",
                        RobotConstants.Shoot.LAUNCH_PITCH_DEG, d, h);
                return s;
            }

            double tNew = Math.sqrt(2.0 * bracket / G_IN_S2);
            boolean converged = Math.abs(tNew - t) < 1e-4;
            t = tNew;
            if (converged) break;
        }

        if (d < 1e-6 || t < 1e-6) {
            s.failReason = "degenerate geometry (muzzle on top of target?)";
            return s;
        }

        // ---- Convert the solution into commands ----------------------------------------------
        double horizontalSpeed = d / t;
        double muzzleSpeed = horizontalSpeed / cosTheta;

        // Empirical drag make-up: the drag-free solution always falls short, more so at range.
        muzzleSpeed *= (1.0 + ball.dragPerInch * d);

        s.muzzleSpeedIn = muzzleSpeed;
        s.flightTimeSec = t;
        s.virtualDistanceIn = d;
        s.fieldAzimuthRad = azimuth;
        s.turretAngleRad = wrapRad(azimuth - robotHeading);
        s.leadAngleRad = wrapRad(azimuth - actualAzimuth);
        s.flywheelRpm = muzzleSpeedToRpm(muzzleSpeed, ball);

        // ---- Sanity checks -------------------------------------------------------------------
        if (s.flywheelRpm > RobotConstants.Shoot.MAX_FLYWHEEL_RPM) {
            s.failReason = String.format("out of range: needs %.0f rpm, max is %.0f",
                    s.flywheelRpm, RobotConstants.Shoot.MAX_FLYWHEEL_RPM);
            return s;
        }
        if (s.turretAngleRad < RobotConstants.Turret.MIN_ANGLE_RAD
                || s.turretAngleRad > RobotConstants.Turret.MAX_ANGLE_RAD) {
            s.failReason = String.format("turret cannot reach %.0f deg -- rotate the chassis",
                    Math.toDegrees(s.turretAngleRad));
            return s;
        }

        s.valid = true;
        return s;
    }

    /**
     * Convenience overload for a stationary shot. Useful for tuning, and as the baseline you
     * compare the moving solution against when you are checking that the lead is sane.
     */
    public static Solution solveStationary(double muzzleX, double muzzleY, double muzzleZ,
                                           double targetX, double targetY, double targetZ,
                                           double robotHeading, BallType ball) {
        return solve(muzzleX, muzzleY, muzzleZ, targetX, targetY, targetZ,
                0, 0, robotHeading, ball);
    }

    /**
     * Ball exit speed (in/s) to flywheel RPM.
     *
     * <p>Flywheel surface speed must be {@code muzzleSpeed / transferEfficiency}, and surface
     * speed is {@code pi * diameter * revsPerSecond}. So:
     * <pre>
     *   rpm = 60 * muzzleSpeed / (efficiency * pi * diameter)
     * </pre>
     * This is where the POLLEN/NECTAR difference enters. Same required ball speed, different
     * efficiency, therefore different RPM -- which is what the colour sensor is for.
     */
    public static double muzzleSpeedToRpm(double muzzleSpeedIn, BallType ball) {
        double eff = ball.transferEfficiency > 1e-6 ? ball.transferEfficiency : 0.5;
        double circumference = Math.PI * RobotConstants.Shoot.FLYWHEEL_DIAMETER_IN;
        return 60.0 * muzzleSpeedIn / (eff * circumference);
    }

    /** Inverse of {@link #muzzleSpeedToRpm}, for the tuning OpMode. */
    public static double rpmToMuzzleSpeed(double rpm, BallType ball) {
        double eff = ball.transferEfficiency > 1e-6 ? ball.transferEfficiency : 0.5;
        double circumference = Math.PI * RobotConstants.Shoot.FLYWHEEL_DIAMETER_IN;
        return rpm * eff * circumference / 60.0;
    }

    /**
     * How far a ball launched flat off a table of height {@code tableHeightIn} would fly at this
     * muzzle speed. This is the measurement {@code T5_ShooterRpmTuner} uses to back out the
     * transfer efficiency, because it needs no vertical aiming and no field.
     */
    public static double flatShotRangeIn(double muzzleSpeedIn, double tableHeightIn) {
        double fallTime = Math.sqrt(2.0 * tableHeightIn / G_IN_S2);
        return muzzleSpeedIn * fallTime;
    }

    /** Back out the transfer efficiency from a measured flat shot. Used by the tuner. */
    public static double efficiencyFromFlatShot(double rpm, double tableHeightIn,
                                                double measuredRangeIn) {
        double fallTime = Math.sqrt(2.0 * tableHeightIn / G_IN_S2);
        double actualMuzzleSpeed = measuredRangeIn / fallTime;
        double surfaceSpeed = rpm / 60.0 * Math.PI * RobotConstants.Shoot.FLYWHEEL_DIAMETER_IN;
        return surfaceSpeed < 1e-6 ? 0 : actualMuzzleSpeed / surfaceSpeed;
    }

    /** Wrap an angle into [-pi, pi]. */
    public static double wrapRad(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
