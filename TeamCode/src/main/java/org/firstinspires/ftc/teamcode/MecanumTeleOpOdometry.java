package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * Your MecanumTeleOp, with odometry and live telemetry added. Same structure, same hardware names,
 * same controls -- plus the instrumentation needed to find out why rotation isn't working.
 *
 * <h2>First: your original code ALREADY has rotation</h2>
 * <pre>
 *   double rx = gamepad1.right_stick_x;            // &lt;-- the rotation input
 *   leftFront.setPower ((y + x + rx) / denominator);
 *   leftBack.setPower  ((y - x + rx) / denominator);
 *   rightFront.setPower((y - x - rx) / denominator);
 *   rightBack.setPower ((y + x - rx) / denominator);
 * </pre>
 * That is the standard Game Manual 0 mecanum mixing, and it is correct for your motor directions
 * (left pair REVERSE, right pair FORWARD). Pushing the <b>RIGHT stick</b> sideways should rotate the
 * robot -- right stick right turns clockwise. So this is not missing code; something is stopping it
 * from reaching the wheels. The telemetry below tells you which.
 *
 * <h2>How to find the fault in about two minutes</h2>
 * <ol>
 *   <li><b>Press START and look at "right_stick_x" in telemetry. Wiggle the right stick.</b>
 *       <ul>
 *         <li><b>Number doesn't move</b> -&gt; it's the gamepad, not the robot. Most likely causes:
 *             a Logitech F310 with its rear <b>X/D switch set to D</b> (it must be <b>X</b>); the
 *             gamepad registered as <b>gamepad2</b> instead of gamepad1 (press
 *             <i>start+a</i> to bind it as driver 1); or the SDK 12.0 case where a pad connects
 *             through Android's generic driver instead of the Driver Station's own USB driver --
 *             SDK 12.0 shows that as an <b>orange</b> gamepad indicator on the Driver Station, and
 *             axis mapping goes wrong. Check the indicator colour.</li>
 *         <li><b>Number moves fine</b> -&gt; the gamepad is good, go to step 2.</li>
 *       </ul></li>
 *   <li><b>Hold Y.</b> That commands a fixed rotation directly, ignoring the sticks entirely.
 *       <ul>
 *         <li><b>Robot rotates on Y but not on the stick</b> -&gt; the mixing and the drivetrain are
 *             fine. Look again at the stick reading and at deadbands.</li>
 *         <li><b>Robot does not rotate on Y either</b> -&gt; it is the drivetrain. Watch the four
 *             "pwr" values in telemetry: they should read roughly +0.3, +0.3, -0.3, -0.3. If the
 *             numbers are right but the robot doesn't turn, it is mechanical -- see step 3.</li>
 *       </ul></li>
 *   <li><b>Mechanical check: your mecanum rollers.</b> Looking down at the robot from above, the top
 *       rollers must form an <b>X</b>. A very common build error is installing them so they form
 *       a <b>diamond</b> (or all parallel), which lets the robot drive and strafe while rotation
 *       fights itself. Lift the robot and spin each wheel by hand to confirm all four actually turn
 *       and nothing is binding.</li>
 * </ol>
 *
 * <h2>And the actual answer to "how do I see real-time data"</h2>
 * One line, and it is the reason your telemetry felt sluggish before:
 * <pre>
 *   telemetry.setMsTransmissionInterval(50);
 * </pre>
 * The FTC default is <b>250 ms</b> -- that is 4 updates per second, no matter how fast your loop
 * runs, and any {@code update()} superseded before the interval expires is simply dropped. At 4 Hz,
 * odometry numbers look like they are lagging or frozen. 50 ms gives you 20 Hz, which reads as live.
 *
 * <p>The {@code loop} and {@code loops} readouts at the bottom of telemetry exist so you can prove
 * the data is live rather than stale: if the counter is climbing, the numbers next to it are fresh.
 *
 * <h2>Important: do NOT reach for FTC Dashboard or Panels</h2>
 * They are the usual answer to this question, and for BIOBUZZ they are <b>explicitly prohibited</b>.
 * From the 2026-2027 Competition Manual, Section 12, rule <b>R704.D</b>, verbatim:
 *
 * <blockquote>"Software may only stream robot control data, debugging data, and telemetry to and from
 * the ROBOT using the FTC Driver Station Application. Additional logging/streaming services, such as
 * those hosted by third party plugins and tools such as <b>FTC Dashboard, FTControl Panels, and
 * others are prohibited</b>. No continuous video stream is allowed."</blockquote>
 *
 * This is new for BIOBUZZ -- last season's rule did not name them -- so older tutorials and forum
 * posts recommending them are now out of date. Driver Station telemetry is the supported path, which
 * is why this OpMode invests in making it good.
 */
@TeleOp(name = "MecanumTeleOp + Odometry", group = "TeleOp")
public class MecanumTeleOpOdometry extends LinearOpMode {

    // Drive Motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    // Intake Motor
    private DcMotor intakeMain = null;

    // Intake Servos
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;

    // Odometry. Nullable on purpose -- see initOdometry().
    private Odometry odometry = null;
    private String odometryError = null;

    /** Ignore stick values smaller than this, so a worn gamepad doesn't creep the robot. */
    private static final double STICK_DEADBAND = 0.05;

    /** Fixed power used by the hold-Y rotation test. */
    private static final double ROTATION_TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        // Hardware Mapping -- unchanged from your original
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftfront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightback");
        leftBack   = hardwareMap.get(DcMotor.class, "leftback");
        intakeMain  = hardwareMap.get(DcMotor.class, "intakemain");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");
        leftIntake  = hardwareMap.get(CRServo.class, "leftintake");

        // Drive motor directions, read from RobotConstants.Drive so there is ONE place to change
        // them. They used to be hardcoded here, which meant editing the constants file did nothing
        // to the OpMode you were actually driving -- a genuinely confusing way to lose an evening.
        leftFront.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_LEFT));
        leftBack.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_LEFT));
        rightFront.setDirection(dir(RobotConstants.Drive.REVERSE_FRONT_RIGHT));
        rightBack.setDirection(dir(RobotConstants.Drive.REVERSE_BACK_RIGHT));

        // Servo directions -- unchanged
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        // Brake mode on drivetrain -- unchanged
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initOdometry();

        // THE line that makes telemetry live. Default is 250 ms = 4 Hz.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData("Status", "Initialized");
        if (odometry != null) {
            telemetry.addLine("Pinpoint found. HOLD THE ROBOT STILL -- the IMU is calibrating.");
        } else {
            telemetry.addLine("No Pinpoint (" + odometryError + ")");
            telemetry.addLine("Driving still works. Add it to the config when it's wired.");
        }
        telemetry.update();

        waitForStart();

        int loops = 0;
        long lastNano = System.nanoTime();
        double loopMs = 0;

        while (opModeIsActive()) {
            loops++;
            long now = System.nanoTime();
            loopMs = (now - lastNano) / 1e6;
            lastNano = now;

            // Read the Pinpoint exactly once per loop, before anything uses it.
            if (odometry != null) {
                odometry.update();
                if (gamepad1.back) odometry.resetPoseAndCalibrateImu();
            }

            // --- DRIVE CODE ---
            // Same maths as yours, with a deadband added so the sticks can't creep.
            double y  = deadband(-gamepad1.left_stick_y);
            double x  = deadband(gamepad1.left_stick_x) * 1.1;   // 1.1 counteracts imperfect strafing
            double rx = deadband(gamepad1.right_stick_x);

            // DIAGNOSTIC: hold Y to command rotation directly, bypassing the sticks entirely.
            // This is the single most useful test for your problem -- it separates "the gamepad
            // isn't giving me a value" from "the drivetrain won't turn".
            boolean rotationTest = gamepad1.y;
            if (rotationTest) {
                y = 0;
                x = 0;
                rx = ROTATION_TEST_POWER;
            }

            // The 1.1 multiplier can push x above 1.0, which is exactly why this denominator is a
            // max() against 1.0 rather than a plain divide: it only scales down when the sum would
            // clip, so normal driving keeps full power and hard diagonals stay proportional.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            // --- INTAKE & SERVO CODE ---
            // Your original drove the motor from (right_trigger - left_trigger) with NO threshold,
            // while the servos used a 0.1 threshold. That meant a trigger resting at 0.03 spun the
            // motor forever while the servos sat still. Both now use the same threshold, so the
            // motor and servos are always in agreement.
            double intakeIn  = gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger : 0.0;
            double intakeOut = gamepad1.left_trigger  > 0.1 ? gamepad1.left_trigger  : 0.0;
            double motorPower = intakeIn - intakeOut;
            intakeMain.setPower(motorPower);

            if (intakeIn > 0) {
                leftIntake.setPower(1.0);
                rightIntake.setPower(1.0);
            } else if (intakeOut > 0) {
                leftIntake.setPower(-1.0);
                rightIntake.setPower(-1.0);
            } else {
                leftIntake.setPower(0.0);
                rightIntake.setPower(0.0);
            }

            // --- TELEMETRY ---
            if (rotationTest) {
                telemetry.addLine(">>> ROTATION TEST ACTIVE (Y held) -- sticks ignored <<<");
                telemetry.addLine("Expect pwr LF/LB positive, RF/RB negative, robot spinning.");
                telemetry.addLine();
            }

            telemetry.addLine("=== GAMEPAD (wiggle the RIGHT stick) ===");
            telemetry.addData("right_stick_x", "%+6.3f   <- THIS is rotation", gamepad1.right_stick_x);
            telemetry.addData("left_stick_y ", "%+6.3f", gamepad1.left_stick_y);
            telemetry.addData("left_stick_x ", "%+6.3f", gamepad1.left_stick_x);
            telemetry.addData("triggers R/L ", "%.2f  %.2f",
                    gamepad1.right_trigger, gamepad1.left_trigger);

            telemetry.addLine();
            telemetry.addLine("=== MIXED INPUTS -> WHEEL POWER ===");
            telemetry.addData("y / x / rx", "%+5.2f  %+5.2f  %+5.2f", y, x, rx);
            telemetry.addData("pwr LF / RF", "%+5.2f  %+5.2f", lfPower, rfPower);
            telemetry.addData("pwr LB / RB", "%+5.2f  %+5.2f", lbPower, rbPower);

            telemetry.addLine();
            telemetry.addLine("=== ODOMETRY ===");
            if (odometry != null) {
                telemetry.addData("Pinpoint", odometry.getStatusText());
                telemetry.addData("X / Y (in)", "%+8.2f  %+8.2f", odometry.getX(), odometry.getY());
                telemetry.addData("Heading (deg)", "%+8.2f", odometry.getHeadingDeg());
                telemetry.addData("Speed (in/s)", "%8.1f", odometry.getSpeed());
                telemetry.addData("Rotation (deg/s)", "%+8.1f",
                        Math.toDegrees(odometry.getHeadingVelRad()));
                telemetry.addData("Pinpoint rate", "%.0f Hz", odometry.getPinpointFrequency());
                telemetry.addLine("Rotating the robot must change Heading. If Heading");
                telemetry.addLine("moves but the robot doesn't, that's a drivetrain fault.");
            } else {
                telemetry.addData("Pinpoint", "NOT AVAILABLE");
                telemetry.addLine(odometryError);
            }

            telemetry.addLine();
            telemetry.addData("loop", "#%d   %.1f ms   (counter climbing = data is live)",
                    loops, loopMs);
            telemetry.addLine("Y = rotation test   BACK = reset odometry pose");
            telemetry.update();
        }

        // Leave nothing powered when the OpMode ends.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        intakeMain.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    /**
     * Bring up odometry without making it a hard requirement.
     *
     * <p>{@code hardwareMap.get} throws if the device isn't in the Robot Configuration, which would
     * stop this OpMode from running at all. Since the Pinpoint may not be wired yet, a failure here
     * is recorded and shown on telemetry instead -- the robot still drives, which matters when the
     * team needs to practise while the odometry pods are being mounted.
     */
    private void initOdometry() {
        try {
            odometry = new Odometry(hardwareMap);
        } catch (Exception e) {
            odometry = null;
            String message = e.getMessage();
            odometryError = "add a goBILDA Pinpoint named \"pinpoint\" to the config"
                    + (message == null ? "" : " [" + message + "]");
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }

    /** Boolean "is reversed" from RobotConstants to the SDK's Direction enum. */
    private static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
