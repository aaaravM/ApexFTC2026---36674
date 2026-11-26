package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Owns the VisionPortal for the monochrome ball camera and converts its sightings into the robot
 * frame. Replaces the colour {@code BallTracker}, which cannot work on a mono sensor.
 *
 * <h2>MJPEG IS NOT OPTIONAL ON THIS CAMERA. Read this before changing the stream format.</h2>
 * VisionPortal defaults to <b>YUY2</b>, and on the B0332 that is a trap with no symptom other than
 * terrible performance:
 *
 * <table>
 *   <tr><th>format</th><th>resolutions offered</th><th>framerate</th></tr>
 *   <tr><td>YUY2 (the default)</td><td>1280x720 and 1280x800 ONLY</td><td><b>10 fps</b></td></tr>
 *   <tr><td>MJPEG</td><td>320x240, 640x480, 800x600, 1280x720, 1280x800</td><td>100-120 fps</td></tr>
 * </table>
 *
 * Two things follow. First, you cannot escape the 10 fps by asking for a smaller image: YUY2 has no
 * small modes at all on this camera, so a request for 640x480 in YUY2 cannot be honoured. Second,
 * the cause is bandwidth, and the arithmetic confirms the spec: USB 2.0 high-speed isochronous
 * carries about 24.6 MB/s, while YUY2 at 1280x800 is 2,048,000 bytes per frame -- a 12 fps ceiling,
 * and the documented 10 fps sits just under it. YUY2 spends 16 bits per pixel to transport a sensor
 * that produces 8 bits of real information, so half the USB budget is literally null chroma.
 *
 * <p>Hence {@code setStreamFormat(MJPEG)} below. With it, <b>1280x800 native runs at 100+ fps</b>,
 * which is both the best resolution and the best framerate available -- there is no trade to make.
 *
 * <h2>Checking what the camera actually gave us</h2>
 * {@link #getActualResolution()} exists because a USB camera will accept a resolution request it
 * cannot honour and quietly return the nearest mode it can do. That is the single most common way a
 * vision pipeline ends up with the wrong focal length, because fx scales with capture width.
 *
 * <p>Not hypothetical here: probing this camera over DirectShow on a laptop, a request for
 * <b>640x400 came back as 640x480</b>, and the 1280 modes measured 10.7 fps because DirectShow had
 * negotiated YUY2 for them. <b>Print {@link #getActualResolution()} at init and compare it against
 * what you asked for.</b>
 *
 * <h2>This exact camera is officially supported</h2>
 * FIRST's own "Webcams for FTC VisionPortal" wiki page lists the Arducam Global Shutter OV9281
 * module by name and links the same Amazon listing the team bought from, so there is no
 * similar-but-different-SKU risk. SDK 8.2 shipped a UVC driver fix specifically for this camera
 * ("Fixes UVC driver compatibility with Arducam OV9281 global shutter camera", in this repo's own
 * README release notes) along with MJPEG payload streaming support. The team is on 12.0, so both
 * are long since included.
 *
 * <h2>What this class does not do yet: temporal tracking</h2>
 * Each call reports what the newest frame sees, with no memory between frames. That leaves two
 * things on the table, and both are known gaps rather than oversights:
 * <ul>
 *   <li><b>Ball velocity</b>, which the moving-shot solver and any intercept planning need.</li>
 *   <li><b>Rejecting a wheel.</b> Single-frame geometry cannot separate a 3.5 in compliant wheel on
 *       the floor from a 3.6 in NECTAR -- same shape, same size, same place, and no colour. FTC
 *       wheel sizes bracket the game pieces almost exactly. The fix is to track in FIELD
 *       coordinates: a wheel moves with its robot, a loose ball does not. See
 *       {@code tools/vision/test_detect.py::test_known_limitation}.</li>
 * </ul>
 */
public class MonoBallTracker {

    private final VisionPortal visionPortal;
    private final MonoBallProcessor processor;
    private final String cameraName;

    /** Uses {@link RobotConstants.Names#WEBCAM_BALLS} and the configured stream resolution. */
    public MonoBallTracker(HardwareMap hardwareMap) {
        // NEGATED: RobotConstants.Camera.PITCH_DEG is UP-positive, this class is DOWN-positive.
        // See MonoBallProcessor.PITCH_SIGN_WARNING -- a sign error here is not recoverable after
        // the fact the way a wrong focal length is.
        this(hardwareMap, RobotConstants.Names.WEBCAM_BALLS,
                RobotConstants.Camera.WIDTH, RobotConstants.Camera.HEIGHT,
                RobotConstants.Camera.Z_IN, -RobotConstants.Camera.PITCH_DEG);
    }

    /**
     * @param cameraName     configuration name of the webcam
     * @param width,height   resolution to REQUEST -- check {@link #getActualResolution()} after
     * @param cameraHeightIn lens centre above the tiles, inches
     * @param pitchDownDeg   <b>DOWN-positive</b>, opposite to RobotConstants.Camera.PITCH_DEG
     */
    public MonoBallTracker(HardwareMap hardwareMap, String cameraName, int width, int height,
                           double cameraHeightIn, double pitchDownDeg) {
        this.cameraName = cameraName;
        this.processor = new MonoBallProcessor(cameraHeightIn, pitchDownDeg);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(width, height))
                .addProcessor(processor);

        // MJPEG, because that is what this camera offers. Probing the B0332 over USB showed it
        // negotiating MJPG rather than a raw mono format -- which is the good case, since the SDK's
        // StreamFormat does not list a GREY/Y8 option and a mono-only camera might not have
        // enumerated at all.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        this.visionPortal = builder.build();
    }

    /**
     * Short fixed exposure with gain to compensate. Call once, after streaming starts; blocks
     * briefly.
     *
     * <p>Worth doing even though this is a global-shutter camera. Global shutter removes
     * rolling-shutter SKEW, but it does not shorten the exposure: a long exposure still smears a
     * moving ball into an ellipse, and an ellipse fails the circularity test. Auto-exposure in gym
     * lighting reliably chooses "too long" for a robot that is driving.
     *
     * <p>It also pins the framerate. Auto-exposure lengthens the exposure in dim light, and the
     * camera then cannot sustain its mode -- measured framerate drops with no other symptom.
     */
    public void setManualExposure() {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            long deadline = System.currentTimeMillis() + 5000;
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                    && System.currentTimeMillis() < deadline) {
                try { Thread.sleep(20); } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposure.setExposure(RobotConstants.Camera.EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        if (gain != null) gain.setGain(RobotConstants.Camera.GAIN);
    }

    /**
     * Every webcam in the robot configuration, as "name (serial)".
     *
     * <p>A wrong camera name throws at init and reads on the Driver Station like a code fault
     * rather than a configuration one. Printing the list turns "it crashed" into "ah, it is called
     * something else" -- which is the single most likely thing to go wrong the first time this is
     * plugged in.
     */
    public static List<String> listWebcams(HardwareMap hardwareMap) {
        List<String> out = new ArrayList<>();
        try {
            for (WebcamName cam : hardwareMap.getAll(WebcamName.class)) {
                StringBuilder sb = new StringBuilder();
                for (String n : hardwareMap.getNamesOf(cam)) {
                    if (sb.length() > 0) sb.append('/');
                    sb.append(n);
                }
                String serial;
                try {
                    serial = cam.getSerialNumber().toString();
                } catch (Exception e) {
                    serial = "?";
                }
                out.add(sb + "  (" + serial + ")");
            }
        } catch (Exception e) {
            out.add("could not enumerate: " + e.getMessage());
        }
        return out;
    }

    /** Resolutions to try, best first. All are real MJPEG modes on the B0332. */
    public static final int[][] FALLBACK_MODES = {
            { 1280, 800 },   // native 16:10, 100-120 fps in MJPEG, the only documented FOV
            { 1280, 720 },
            { 800, 600 },
            { 640, 480 },
            { 320, 240 },
    };

    /** Result of {@link #openWithFallback}: the tracker, or why every mode failed. */
    public static final class OpenResult {
        public final MonoBallTracker tracker;   // null on failure
        public final String log;                // what was tried, and what happened

        OpenResult(MonoBallTracker tracker, String log) {
            this.tracker = tracker;
            this.log = log;
        }

        public boolean ok() { return tracker != null; }
    }

    /**
     * Open the camera, trying each resolution in turn until one actually streams.
     *
     * <h3>Why a fallback chain rather than one request</h3>
     * {@code VisionPortal.build()} returns immediately and opens the camera on a background
     * thread, so an unsupported resolution does not throw -- the portal simply transitions to
     * {@code ERROR} a moment later, and an OpMode that never checks sits there with no detections
     * and no explanation. Worse, a camera that accepts the request and substitutes a different mode
     * leaves the intrinsics wrong rather than absent.
     *
     * <p>So this waits for a definite STREAMING or ERROR for each candidate, closes and moves on if
     * it failed, and returns a log of exactly what happened. On success the caller should still
     * compare {@link #getActualResolution()} against what it asked for.
     *
     * @param modes      resolutions to try, best first; pass {@link #FALLBACK_MODES} for the default
     * @param timeoutMs  how long to wait for each candidate to declare itself
     */
    public static OpenResult openWithFallback(HardwareMap hardwareMap, String cameraName,
                                              int[][] modes, double cameraHeightIn,
                                              double pitchDownDeg, long timeoutMs) {
        StringBuilder log = new StringBuilder();

        List<String> cams = listWebcams(hardwareMap);
        log.append("webcams in config: ").append(cams.isEmpty() ? "NONE" : cams.toString())
           .append('\n');

        for (int[] mode : modes) {
            int w = mode[0], h = mode[1];
            MonoBallTracker t = null;
            try {
                t = new MonoBallTracker(hardwareMap, cameraName, w, h,
                        cameraHeightIn, pitchDownDeg);
                VisionPortal portal = t.getVisionPortal();

                long deadline = System.currentTimeMillis() + timeoutMs;
                VisionPortal.CameraState state = portal.getCameraState();
                while (state != VisionPortal.CameraState.STREAMING
                        && state != VisionPortal.CameraState.ERROR
                        && System.currentTimeMillis() < deadline) {
                    try {
                        Thread.sleep(25);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                    state = portal.getCameraState();
                }

                if (state == VisionPortal.CameraState.STREAMING) {
                    log.append(String.format("%dx%d STREAMING -> using it%n", w, h));
                    return new OpenResult(t, log.toString());
                }
                log.append(String.format("%dx%d gave %s, trying next%n", w, h, state));
                t.close();
            } catch (Exception e) {
                log.append(String.format("%dx%d threw %s: %s%n", w, h,
                        e.getClass().getSimpleName(), e.getMessage()));
                if (t != null) {
                    try { t.close(); } catch (Exception ignored) { }
                }
            }
        }

        log.append("every mode failed. Check the camera name above against what this OpMode asks\n");
        log.append("for, check the USB cable carries data, and check the Hub has power.\n");
        return new OpenResult(null, log.toString());
    }

    /** One ball, positioned in the ROBOT frame. */
    public static final class Ball {
        public final MonoBallProcessor.Sighting sighting;
        /** Robot frame, inches. +X forward, +Y left. */
        public final double xIn, yIn;
        /** Horizontal distance from the robot's tracking point, inches. */
        public final double rangeIn;
        /** Bearing in the ROBOT frame, radians CCW. This is what a drive or turret target wants. */
        public final double bearingRad;

        Ball(MonoBallProcessor.Sighting s, BallGeometry.Position p) {
            this.sighting = s;
            this.xIn = p.xIn; this.yIn = p.yIn; this.rangeIn = p.rangeIn;
            this.bearingRad = p.bearingRad();
        }

        @Override public String toString() {
            return String.format("%s at (%.1f, %.1f) range %.1fin bearing %+.1fdeg",
                    sighting.type, xIn, yIn, rangeIn, Math.toDegrees(bearingRad));
        }
    }

    /**
     * Newest sightings converted into the robot frame.
     *
     * @param turretAngleRadAtCapture the turret angle <b>when the frame was captured</b>, not now.
     *                                Ignored when the camera is chassis-mounted. On a slewing
     *                                turret the difference is real: at 160 deg/s and 30 ms of
     *                                latency the turret has moved almost 5 degrees, which at 60 in
     *                                is 5 in of lateral error on the ball's position.
     */
    public List<Ball> getBalls(double turretAngleRadAtCapture) {
        BallGeometry.CameraPose pose = poseFor(turretAngleRadAtCapture);
        List<MonoBallProcessor.Sighting> raw = processor.getSightings();
        List<Ball> out = new ArrayList<>(raw.size());
        for (MonoBallProcessor.Sighting s : raw) {
            out.add(new Ball(s, s.toRobotFrame(pose)));
        }
        return out;
    }

    /** Nearest ball in the robot frame, or null. */
    public Ball getClosest(double turretAngleRadAtCapture) {
        List<Ball> balls = getBalls(turretAngleRadAtCapture);
        return balls.isEmpty() ? null : balls.get(0);
    }

    /** The camera pose for a given turret angle, built from RobotConstants. */
    public static BallGeometry.CameraPose poseFor(double turretAngleRad) {
        return new BallGeometry.CameraPose(
                RobotConstants.Camera.Z_IN,
                // NEGATED: CameraPose is DOWN-positive, PITCH_DEG is UP-positive.
                -RobotConstants.Camera.PITCH_DEG,
                RobotConstants.Camera.X_IN,
                RobotConstants.Camera.Y_IN,
                RobotConstants.Camera.MOUNTED_ON_TURRET ? turretAngleRad : 0.0,
                RobotConstants.Camera.TURRET_PIVOT_X_IN,
                RobotConstants.Camera.TURRET_PIVOT_Y_IN,
                RobotConstants.Camera.MOUNTED_ON_TURRET);
    }

    /**
     * The resolution the camera ACTUALLY gave us, which may not be what was requested.
     *
     * @return "WxH", or a note if it is not known yet
     */
    public String getActualResolution() {
        if (visionPortal == null) return "no portal";
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return "not streaming yet (" + visionPortal.getCameraState() + ")";
        }
        BallGeometry.Intrinsics intr = processor.getIntrinsics();
        return intr == null ? "unknown" : intr.width + "x" + intr.height;
    }

    /** One line summarising whether this pipeline can be trusted at all. Put it in telemetry. */
    public String getHealthLine() {
        BallGeometry.Intrinsics intr = processor.getIntrinsics();
        if (intr == null) return "not initialised";
        return String.format("%s | %s | fps %.1f | %.1f ms | hfov %.0fdeg",
                getActualResolution(),
                processor.isCalibrationFallback() ? "*** UNCALIBRATED ***" : "calibrated",
                visionPortal.getFps(), processor.getProcessMs(), intr.hfovDeg());
    }

    public MonoBallProcessor getProcessor() { return processor; }
    public VisionPortal getVisionPortal() { return visionPortal; }
    public String getCameraName() { return cameraName; }

    public void setEnabled(boolean enabled) {
        if (visionPortal != null) visionPortal.setProcessorEnabled(processor, enabled);
    }

    /** Stop streaming to give the CPU back without tearing the portal down. */
    public void stopStreaming() { if (visionPortal != null) visionPortal.stopStreaming(); }

    public void resumeStreaming() { if (visionPortal != null) visionPortal.resumeStreaming(); }

    public void close() { if (visionPortal != null) visionPortal.close(); }
}
