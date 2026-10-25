package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Registers Pedro's AutoTune procedures. Open http://192.168.43.1:10158 while connected to the
 * robot's Wi-Fi to run them.
 *
 * All four use the SAME drivetrain / Pinpoint / Foresight settings as the real Follower
 * (via PedroConstants and FollowerTuning), so what you test here is what runs in matches.
 */
public class Tuning {

  /** Finds drive motor directions. Wheels off the ground. */
  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  /** Finds Pinpoint pod directions and offsets. Push the robot by hand. */
  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  /** Tunes path following. The robot drives itself: needs space ahead and to the left. */
  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        hardwareMap -> new PinpointLocalizer(hardwareMap, PedroConstants.localizerConfig()),
        hardwareMap -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig())
    );
  }

  /** Driving, localization, hold, line, and curve tests using the current settings. */
  @Tuner
  public static Procedure tests() {
    return new Tests(
        hardwareMap -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig()),
        hardwareMap -> new PinpointLocalizer(hardwareMap, PedroConstants.localizerConfig()),
        () -> new Foresight(FollowerTuning.build())
    );
  }
}