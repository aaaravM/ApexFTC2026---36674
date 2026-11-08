package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BallType;
import org.firstinspires.ftc.teamcode.vision.BallTracker;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * STEP 7. The first thing to run with the new Logitech C270. Finds POLLEN and NECTAR by colour and
 * reports how far away and in which direction each one is.
 *
 * <h2>Before you run it</h2>
 * Add the webcam to the Robot Configuration: <b>Configure Robot -&gt; USB Devices -&gt; Webcam</b>,
 * named exactly <b>{@code Webcam 1}</b> (that is {@link #CAMERA_NAME} below). Plug it into the
 * Control Hub's <b>USB 2.0</b> Type-A port, leaving the blue USB 3.0 port free for the Limelight.
 *
 * <h2>Watch the picture, not just the numbers</h2>
 * On the Driver Station, <b>three-dot menu -&gt; Camera Stream</b> shows the live image with a
 * yellow circle drawn around every blob the detector accepted. That view is worth far more than the
 * numbers while tuning, because you can see at a glance whether it is locking onto balls or onto
 * somebody's shoe. The stream runs during INIT too, so you can aim the camera before pressing START.
 *
 * <h2>What to check, in order</h2>
 * <ol>
 *   <li><b>Camera state says STREAMING.</b> If it says ERROR, the webcam is not in the
 *       configuration under that exact name, or the cable is not in a Type-A host port. Nothing
 *       below works until this is right.</li>
 *   <li><b>Hold up a POLLEN (yellow) ball.</b> It should appear in the list. If it does not, open
 *       the Camera Stream: if the ball is clearly visible but no circle is drawn, the colour
 *       threshold or the circularity filter is rejecting it.</li>
 *   <li><b>Check the distance.</b> Put a ball exactly 24 inches from the lens and read the
 *       reported figure -- it should be within an inch or two.
 *       <p>If it is off by a consistent <i>factor</i> rather than a constant, the focal length does
 *       not match the capture resolution. That was a real bug here until recently, so it is worth
 *       confirming: the C270's calibrated focal length is 822.317 px at 640x480, which scales to
 *       411.16 px at the 320x240 this actually runs at.</li>
 *   <li><b>Check the bearing sign.</b> Move the ball to the LEFT of frame. Bearing should go
 *       <b>positive</b>. Everything in this codebase uses "positive is left", so if it goes negative
 *       the sign needs fixing before the intake ever chases a ball.</li>
 *   <li><b>Try NECTAR</b> (red and blue). Colour is what tells us which ball it is, and that is
 *       what makes distance possible at all -- a near POLLEN and a far NECTAR look identical
 *       otherwise.</li>
 * </ol>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li><b>A</b> / cross: toggle POLLEN</li>
 *   <li><b>B</b> / circle: toggle RED NECTAR</li>
 *   <li><b>X</b> / square: toggle BLUE NECTAR</li>
 * </ul>
 * Each colour is a separate pass over the image, so switching off the ones you do not need buys back
 * real loop time. In a match you only ever need POLLEN plus your own alliance's NECTAR.
 */
@TeleOp(name = "T8 Ball Camera Test (C270)", group = "Tuning")
public class T8_BallCameraTest extends LinearOpMode {

    /** Must match the Robot Configuration exactly. Case-sensitive. */
    public static final String CAMERA_NAME = "Webcam 1";

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        BallTracker tracker;
        try {
            tracker = new BallTracker(hardwareMap, CAMERA_NAME,
                    BallType.POLLEN, BallType.NECTAR_RED, BallType.NECTAR_BLUE);
        } catch (Exception e) {
            // Report the actual problem instead of dying with a stack trace nobody reads.
            while (opModeInInit() || opModeIsActive()) {
                telemetry.addLine("CAMERA NOT FOUND");
                telemetry.addLine();
                telemetry.addData("Looked for", CAMERA_NAME);
                telemetry.addLine();
                telemetry.addLine("Fix: Configure Robot -> USB Devices -> Webcam,");
                telemetry.addLine("named exactly that, then Save AND Activate.");
                telemetry.addLine("Use a Type-A port, not the USB-C port.");
                telemetry.addLine();
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                sleep(200);
            }
            return;
        }

        boolean wantPollen = true, wantRed = true, wantBlue = true;
        boolean aWas = false, bWas = false, xWas = false;

        telemetry.addLine("Camera opening. Aim it with the Camera Stream");
        telemetry.addLine("(three dots -> Camera Stream), then press START.");
        telemetry.update();
        waitForStart();

        int loops = 0;
        long lastNano = System.nanoTime();

        while (opModeIsActive()) {
            loops++;
            double loopMs = (System.nanoTime() - lastNano) / 1e6;
            lastNano = System.nanoTime();

            if (gamepad1.a && !aWas) {
                wantPollen = !wantPollen;
                tracker.setEnabled(BallType.POLLEN, wantPollen);
            }
            aWas = gamepad1.a;
            if (gamepad1.b && !bWas) {
                wantRed = !wantRed;
                tracker.setEnabled(BallType.NECTAR_RED, wantRed);
            }
            bWas = gamepad1.b;
            if (gamepad1.x && !xWas) {
                wantBlue = !wantBlue;
                tracker.setEnabled(BallType.NECTAR_BLUE, wantBlue);
            }
            xWas = gamepad1.x;

            tracker.update();

            VisionPortal.CameraState state = tracker.getVisionPortal().getCameraState();
            telemetry.addData("Camera", state);
            if (state != VisionPortal.CameraState.STREAMING) {
                telemetry.addLine("Not streaming yet -- wait, or check the config name.");
            }
            telemetry.addData("Looking for", "%s%s%s",
                    wantPollen ? "POLLEN " : "", wantRed ? "RED " : "", wantBlue ? "BLUE" : "");

            List<BallTracker.Sighting> seen = tracker.getSightings();
            telemetry.addLine();
            telemetry.addData("Balls seen", seen.size());
            telemetry.addLine("  type        dist   bearing radius circ");
            for (BallTracker.Sighting s : seen) {
                telemetry.addLine(String.format("  %-11s %5.1fin %+6.1f%s %4.0fpx %.2f",
                        s.type, s.distanceIn, s.getBearingDeg(),
                        s.getBearingDeg() >= 0 ? "L" : "R", s.radiusPx, s.circularity));
            }

            BallTracker.Sighting closest = tracker.getClosest();
            telemetry.addLine();
            if (closest != null) {
                telemetry.addLine("=== NEAREST ===");
                telemetry.addData("type", closest.type);
                telemetry.addData("distance", "%.1f in", closest.distanceIn);
                telemetry.addData("bearing", "%+.1f deg  (positive = LEFT)", closest.getBearingDeg());
                telemetry.addData("robot x/y", "%+.1f  %+.1f in", closest.robotX, closest.robotY);
            } else {
                telemetry.addLine("(nothing detected -- check the Camera Stream)");
            }

            telemetry.addLine();
            telemetry.addData("loop", "#%d  %.1f ms", loops, loopMs);
            telemetry.addLine("A=POLLEN  B=RED  X=BLUE");
            telemetry.update();
        }

        tracker.close();
    }
}
