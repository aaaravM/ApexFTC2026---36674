package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.util.Ballistics;

/**
 * STEP 5. Measures the one number per ball type that turns the ballistics solution into an RPM:
 * {@link BallType#transferEfficiency}, the fraction of flywheel surface speed the ball leaves with.
 *
 * <h2>The flat-shot method</h2>
 * Aiming a real arc at a real CELL couples together the hood angle, the turret, the range estimate
 * and the efficiency, so a miss tells you nothing about which one was wrong. Instead, shoot
 * <b>horizontally off a table</b>:
 * <ul>
 *   <li>The ball leaves with a purely horizontal speed {@code v}.</li>
 *   <li>It falls the table height {@code h} in time {@code t = sqrt(2h/g)}, independent of {@code v}.</li>
 *   <li>So the distance it lands from the table edge is exactly {@code v * t}, and
 *       {@code v = range / t}.</li>
 * </ul>
 * One tape measure gives you the true muzzle speed with no aiming involved. Divide by the flywheel's
 * surface speed and you have the efficiency.
 *
 * <h2>Procedure</h2>
 * <ol>
 *   <li>Set the shooter on a table with the hood temporarily level -- or accept a small error and
 *       just hold the robot so the muzzle points level. Measure the muzzle height above the floor
 *       and put it in {@code TABLE_HEIGHT_IN} below.</li>
 *   <li>Clear a lane. Press START.</li>
 *   <li>D-pad up/down changes the commanded RPM. Wait for READY.</li>
 *   <li>Fire one ball. Measure where it first hits the floor, horizontally from the muzzle.</li>
 *   <li>Enter that distance with bumpers, press <b>A</b>, and read the computed efficiency.</li>
 *   <li>Repeat at two or three RPMs. The efficiency should come out roughly the same each time; if
 *       it climbs with RPM, your flywheel is slipping at low speed. Put the average into
 *       {@link BallType}.</li>
 *   <li>Do the whole thing again with a NECTAR. Expect a lower number -- it is heavier and gets
 *       compressed harder.</li>
 * </ol>
 *
 * <h2>Then, and only then, tune drag</h2>
 * With efficiency set, aim real arcs at real distances. If the ball consistently falls short and
 * more so at long range, raise {@link BallType#dragPerInch} a little. Drag is the only thing that
 * should be soaking up a range-dependent error, so do not tune it until efficiency is right, or you
 * will be using one wrong number to hide another.
 */
@TeleOp(name = "T5 Shooter RPM Tuner", group = "Tuning")
public class T5_ShooterRpmTuner extends LinearOpMode {

    /** Muzzle height above the floor for the flat-shot test, inches. MEASURE THIS. */
    private static final double TABLE_HEIGHT_IN = 36.0;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Clear the lane in front of the robot. Press START.");
        telemetry.update();
        waitForStart();

        double commandedRpm = 1500;
        double measuredRangeIn = 100;
        BallType ballType = BallType.POLLEN;
        double computedEfficiency = 0;

        boolean upWas = false, downWas = false, lbWas = false, rbWas = false;
        boolean aWas = false, xWas = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upWas) commandedRpm += 100;
            upWas = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !downWas) commandedRpm = Math.max(0, commandedRpm - 100);
            downWas = gamepad1.dpad_down;

            if (gamepad1.right_bumper && !rbWas) measuredRangeIn += 1;
            rbWas = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lbWas) measuredRangeIn = Math.max(0, measuredRangeIn - 1);
            lbWas = gamepad1.left_bumper;

            if (gamepad1.x && !xWas) {
                ballType = (ballType == BallType.POLLEN) ? BallType.NECTAR_RED : BallType.POLLEN;
            }
            xWas = gamepad1.x;

            if (gamepad1.a && !aWas) {
                computedEfficiency = Ballistics.efficiencyFromFlatShot(
                        commandedRpm, TABLE_HEIGHT_IN, measuredRangeIn);
            }
            aWas = gamepad1.a;

            shooter.setTargetRpm(gamepad1.b ? 0 : commandedRpm);

            telemetry.addData("Ball type", "%s  (X to swap)", ballType);
            telemetry.addLine();
            telemetry.addData("Commanded RPM", "%.0f   (dpad up/down)", commandedRpm);
            telemetry.addData("Actual RPM", "%.0f", shooter.getCurrentRpm());
            telemetry.addData("Error", "%+.0f", shooter.getRpmError());
            telemetry.addData("State", shooter.isAtSpeed() ? ">>> READY -- FIRE <<<" : "spinning up");
            telemetry.addLine();
            telemetry.addData("Muzzle height", "%.1f in  (edit TABLE_HEIGHT_IN)", TABLE_HEIGHT_IN);
            telemetry.addData("Measured range", "%.0f in   (bumpers)", measuredRangeIn);
            telemetry.addLine();
            telemetry.addLine("A = compute efficiency from the numbers above");
            telemetry.addLine("B = hold to stop the flywheel");

            if (computedEfficiency > 0) {
                telemetry.addLine();
                telemetry.addLine("=== RESULT ===");
                telemetry.addData("transferEfficiency", "%.3f", computedEfficiency);
                telemetry.addData("implied muzzle speed", "%.1f in/s",
                        measuredRangeIn / Math.sqrt(2 * TABLE_HEIGHT_IN / Ballistics.G_IN_S2));
                telemetry.addLine("Put this in BallType." + ballType.name());
                if (computedEfficiency > 1.05) {
                    telemetry.addLine("Over 1.0 is impossible -- check FLYWHEEL_DIAMETER_IN");
                    telemetry.addLine("and FLYWHEEL_TICKS_PER_REV.");
                }
            }

            telemetry.addLine();
            telemetry.addLine("--- current constants predict ---");
            telemetry.addData("muzzle speed at this RPM", "%.1f in/s",
                    Ballistics.rpmToMuzzleSpeed(commandedRpm, ballType));
            telemetry.addData("flat shot would fly", "%.0f in",
                    Ballistics.flatShotRangeIn(
                            Ballistics.rpmToMuzzleSpeed(commandedRpm, ballType), TABLE_HEIGHT_IN));
            telemetry.update();
        }

        shooter.stop();
    }
}
