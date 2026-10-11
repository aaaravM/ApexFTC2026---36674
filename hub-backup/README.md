# Control Hub backup

Pulled off the Control Hub with `adb pull /sdcard/FIRST` before the first Android Studio / Gradle
deploy replaced the factory Robot Controller app.

Only the two irreplaceable files are kept here. The rest of that pull (OnBot Java's build output,
jar caches, the SDK's own sample templates, match logs) is all regenerable and was left out.

| File | What it is |
|---|---|
| `firstconfig.xml` | The robot configuration — every motor, servo and sensor name |
| `basicteleop.blk` | A Blocks program that was on the Hub |

## Why the configuration file is worth committing

It is the authoritative record of what is actually plugged into the robot, and it settles naming
arguments instantly. It already caught one real bug: the Pinpoint is named **`odomcomp`**, not
`pinpoint`, which is what `RobotConstants` had originally guessed. Every odometry OpMode would have
thrown at init with "device not found", pointing you at the I2C wiring instead of at a string.

Current contents:

```xml
<LynxModule name="Expansion Hub 2" port="2">
    <goBILDA5202SeriesMotor name="intakemain" port="0" />
    <goBILDA5202SeriesMotor name="Flywheel"   port="1" />
</LynxModule>
<LynxModule name="Control Hub" port="173">
    <goBILDA5202SeriesMotor  name="rightfront"  port="0" />
    <goBILDA5202SeriesMotor  name="leftfront"   port="1" />
    <goBILDA5202SeriesMotor  name="rightback"   port="2" />
    <goBILDA5202SeriesMotor  name="leftback"    port="3" />
    <ContinuousRotationServo name="rightintake" port="0" />
    <ContinuousRotationServo name="leftintake"  port="1" />
    <ControlHubImuBHI260AP   name="imu"      port="0" bus="0" />
    <goBILDAPinpoint         name="odomcomp" port="0" bus="1" />
</LynxModule>
```

Note the Pinpoint sits on **bus 1** while the Control Hub's built-in IMU occupies **bus 0** — which
is exactly why the Pinpoint must never be configured on bus 0.

**Keep this updated.** Whenever the robot configuration changes, re-pull it:

```cmd
adb pull /sdcard/FIRST/firstconfig.xml hub-backup\firstconfig.xml
```

Then commit, so the repo and the robot never disagree about a device name.

## About the Blocks program

`basicteleop.blk` is Blocks' own XML format, not Java. It is kept because it cannot be regenerated,
but it is not used by anything in this project — Blocks programs live on the Hub and are edited in
the browser, and the two OnBot Java programs it predates are already ported into `TeamCode/` as
`MecanumTeleOp.java` and `IntakeFlywheelTest.java`.

To restore it: copy it back to `/sdcard/FIRST/blocks/` with `adb push`, or paste it in through the
Blocks editor.

## The full backup

The complete 122-file pull is at `C:\FTC-backup` on the machine this was done from. It is not
committed because almost all of it is regenerable build output. OnBot Java also keeps its own version
history in `/sdcard/FIRST/java/srcBackups/` as timestamped zips — there were 19 of them going back to
August, which is a nicer safety net than most teams realise they have.
