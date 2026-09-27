package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.util.Ballistics;

/**
 * Your intake + flywheel test, but with the flywheel on <b>closed-loop RPM</b> instead of
 * {@code setPower(1.0)} -- and with telemetry that tells you how far the ball will actually go.
 *
 * <h2>Why not just setPower(1.0)?</h2>
 * Three reasons, in increasing order of how much they will cost you:
 *
 * <ol>
 *   <li><b>You are running about twice as fast as the game needs.</b> Full power on a 1:1 6000 RPM
 *       motor is ~6000 RPM. A 14 ft BIOBUZZ shot needs only about 7.3 m/s of ball exit speed, which
 *       is roughly <b>2900 RPM</b> on a 4 in wheel. The whole game lives in <b>1600-2900 RPM</b>.
 *       At 6000 the ball sails well over the CELL, and no amount of aiming fixes that.</li>
 *   <li><b>A power number is not a speed.</b> {@code setPower(1.0)} gives a different RPM at 13.5 V
 *       than at 11.8 V, and your battery will be somewhere in between by the third match. Commanding
 *       an RPM and letting the Hub hold it removes the battery from the equation entirely.</li>
 *   <li><b>The wheel slows on every shot.</b> Open loop, it just stays slow. Closed loop, the
 *       controller pushes it back. That is the difference between the first shot of a burst landing
 *       and all three landing.</li>
 * </ol>
 *
 * <p>There is a happy accident in your motor choice worth knowing: a brushed DC motor makes peak
 * mechanical power at exactly <b>50% of free speed</b>. 2900 RPM is 48% of 6000. So a bare 1:1 5203
 * held at the speed this game actually needs is sitting essentially on its peak-power point --
 * 22.6 W of the 22.64 W available. <b>Do not belt this motor</b>; any gearing moves it off that point
 * and makes shot-to-shot recovery worse.
 *
 * <h2>Controls -- your bindings, kept</h2>
 * <ul>
 *   <li><b>Cross</b>: intake in. <b>Triangle</b>: intake out.</li>
 *   <li><b>Circle</b> (hold): spin the flywheel to the commanded RPM.</li>
 *   <li><b>D-pad up/down</b>: commanded RPM, 100 at a time. <b>D-pad left/right</b>: 25 at a time.</li>
 *   <li><b>Square</b>: switch the predicted-range readout between POLLEN and NECTAR.</li>
 * </ul>
 *
 * <h2>How to use it</h2>
 * Set an RPM, hold Circle, wait for <b>READY</b>, then feed one ball and see where it lands. The
 * telemetry shows the range this RPM <i>should</i> produce. When measured and predicted disagree, the
 * gap is almost entirely {@link BallType#transferEfficiency} -- adjust that, not the RPM, because
 * efficiency is the number that makes every other distance correct too.
 *
 * <p>Watch the <b>current</b> readout while it spins up. Published stall current is 9.2 A, and a
 * step-change setpoint draws close to that for over a second. This OpMode ramps the setpoint for
 * exactly that reason; if you see sustained current near 9 A, something is rubbing.
 */
@TeleOp(name = "Intake + Flywheel Tuner (RPM)", group = "Test")
public class IntakeFlywheelTuner extends LinearOpMode {

    private DcMotorEx intakeMain;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private Shooter shooter;

    /** Muzzle height above the floor, inches. MEASURE THIS on your robot. */
    private static final double MUZZLE_HEIGHT_IN = 14.0;

    @Override
    public void runOpMode() {
        intakeMain  = hardwareMap.get(DcMotorEx.class, RobotConstants.Names.INTAKE);
        leftIntake  = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_L);
        rightIntake = hardwareMap.get(CRServo.class, RobotConstants.Names.INTAKE_SERVO_R);

        // Set the run mode EXPLICITLY. It is sticky in the hub across OpModes until a power cycle,
        // so a tuning OpMode that left this in RUN_USING_ENCODER would change how setPower behaves
        // here, with nothing on screen to explain why.
        intakeMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Resolve the servo-inversion inconsistency between your two files ONE way, here: use
        // setDirection, then always send both servos the same power. Your other test negates the
        // right servo's power instead; both work alone, but doing it in two different places is how
        // you end up inverting it twice.
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        shooter = new Shooter(hardwareMap);

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Clear the lane in front of the robot.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        double commandedRpm = 2400;   // a sensible mid-band starting point, not full power
        BallType ballType = BallType.POLLEN;
        boolean upWas = false, downWas = false, leftWas = false, rightWas = false, squareWas = false;

        while (opModeIsActive()) {
            // --- RPM setpoint ---------------------------------------------------------------
            if (gamepad1.dpad_up && !upWas) commandedRpm += 100;
            upWas = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !downWas) commandedRpm = Math.max(0, commandedRpm - 100);
            downWas = gamepad1.dpad_down;
            if (gamepad1.dpad_right && !rightWas) commandedRpm += 25;
            rightWas = gamepad1.dpad_right;
            if (gamepad1.dpad_left && !leftWas) commandedRpm = Math.max(0, commandedRpm - 25);
            leftWas = gamepad1.dpad_left;

            if (gamepad1.square && !squareWas) {
                ballType = (ballType == BallType.POLLEN) ? BallType.NECTAR_RED : BallType.POLLEN;
            }
            squareWas = gamepad1.square;

            // --- intake (your bindings) -----------------------------------------------------
            if (gamepad1.cross) {
                intakeMain.setPower(1.0);
                leftIntake.setPower(1.0);
                rightIntake.setPower(1.0);
            } else if (gamepad1.triangle) {
                intakeMain.setPower(-1.0);
                leftIntake.setPower(-1.0);
                rightIntake.setPower(-1.0);
            } else {
                intakeMain.setPower(0.0);
                leftIntake.setPower(0.0);
                rightIntake.setPower(0.0);
            }

            // --- flywheel -------------------------------------------------------------------
            // setTargetRpm ramps internally, so hold Circle and let it climb.
            shooter.setTargetRpm(gamepad1.circle ? commandedRpm : 0);

            // --- telemetry ------------------------------------------------------------------
            double actualRpm = shooter.getCurrentRpm();
            double muzzleSpeed = Ballistics.rpmToMuzzleSpeed(actualRpm, ballType);

            telemetry.addData("Ball", "%s   (Square to swap)", ballType);
            telemetry.addLine();
            telemetry.addLine("=== FLYWHEEL ===");
            telemetry.addData("Commanded", "%.0f rpm   (dpad up/down 100, left/right 25)", commandedRpm);
            telemetry.addData("Ramping to", "%.0f rpm", shooter.getCommandedRpm());
            telemetry.addData("Actual", "%.0f rpm  (averaged over %d samples)",
                    actualRpm, RobotConstants.Shoot.RPM_SAMPLE_COUNT);
            telemetry.addData("Error", "%+.0f rpm", shooter.getRpmError());
            telemetry.addData("State", shooter.isAtSpeed() ? ">>> READY -- FEED A BALL <<<"
                    : (gamepad1.circle ? "spinning up" : "hold CIRCLE to spin"));
            telemetry.addData("Current", "%.2f A   (9.2 A = stall)", shooter.getCurrentAmps());

            telemetry.addLine();
            telemetry.addLine("=== WHAT THIS RPM SHOULD DO ===");
            telemetry.addData("ball exit speed", "%.0f in/s  (%.1f m/s)",
                    muzzleSpeed, muzzleSpeed * 0.0254);
            telemetry.addData("flat off a table", "%.0f in from a %.0f in height",
                    Ballistics.flatShotRangeIn(muzzleSpeed, MUZZLE_HEIGHT_IN), MUZZLE_HEIGHT_IN);

            // Reachable range at the configured hood angle, drag-free. Compare against where the
            // ball actually lands -- the difference is drag plus efficiency error.
            Ballistics.Solution solution = Ballistics.solve(
                    0, 0, MUZZLE_HEIGHT_IN,
                    estimateRangeForRpm(actualRpm, ballType), 0, 36.0,
                    0, 0, 0, ballType);
            telemetry.addLine(String.format("at a %.0f deg hood:",
                    RobotConstants.Shoot.LAUNCH_PITCH_DEG));
            telemetry.addData("  reaches about", "%.0f in to a 36 in CELL",
                    estimateRangeForRpm(actualRpm, ballType));
            telemetry.addData("  flight time", solution.valid
                    ? String.format("%.2f s", solution.flightTimeSec) : solution.failReason);

            telemetry.addLine();
            telemetry.addLine("The game needs 1600-2900 rpm. If you are at 6000,");
            telemetry.addLine("the ball is flying well over the CELL.");
            telemetry.addLine();
            telemetry.addLine("Cross = intake in   Triangle = intake out   Circle = shoot");
            telemetry.update();
        }

        shooter.stop();
        intakeMain.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    /**
     * Horizontal distance this RPM reaches, drag-free, at the configured hood angle and a 36 in
     * target height. Inverts the ballistics relation rather than searching.
     *
     * <p>Derivation: at a fixed hood angle {@code theta}, {@code d*tan(theta) - h = G*t^2/2} and
     * {@code d = v*cos(theta)*t}. Eliminating {@code t} gives
     * {@code d = v^2*sin(2*theta)/(2*G) * (1 + sqrt(1 - 2*G*h/(v*sin(theta))^2))}.
     */
    private static double estimateRangeForRpm(double rpm, BallType ball) {
        double v = Ballistics.rpmToMuzzleSpeed(rpm, ball);
        if (v <= 1) return 0;
        double theta = Math.toRadians(RobotConstants.Shoot.LAUNCH_PITCH_DEG);
        double h = 36.0 - MUZZLE_HEIGHT_IN;   // rise from muzzle to CELL opening
        double vSin = v * Math.sin(theta);
        double discriminant = 1 - 2 * Ballistics.G_IN_S2 * h / (vSin * vSin);
        if (discriminant < 0) return 0;       // cannot reach that height at all
        return (v * v * Math.sin(2 * theta)) / (2 * Ballistics.G_IN_S2)
                * (1 + Math.sqrt(discriminant));
    }
}
