# The 2026-10-11 rewire

## No Java code changed, and that is not luck

`hardwareMap.get(DcMotorEx.class, "leftfront")` looks a device up **by name**. The port number only
exists inside the Robot Configuration. So as long as the names stay the same, you can move every
motor to a different port and not touch a single line of Java.

This is worth knowing deliberately: it means **rewiring is a Driver Station job, not a programming
job.** If someone ever tells you a rewire needs code changes, the names changed and they should not
have.

## New layout

| Hub | Port | Name | What it is |
|---|---|---|---|
| Expansion | motor 1 | `leftfront` | drivetrain |
| Expansion | motor 2 | `intakemain` | intake |
| Expansion | motor 3 | `leftback` | drivetrain |
| Expansion | servo 5 | `leftintake` | intake |
| Control | motor 0 | `rightback` | drivetrain |
| Control | motor 1 | `rightfront` | drivetrain |
| Control | motor 3 | `Flywheel` | shooter (moved here in the rewire) |
| Control | servo 1 | `rightintake` | intake |
| Control | I2C bus **?** | `odomcomp` | Pinpoint — **bus changed, add on the DS** |
| Control | I2C bus 0 | `imu` | built-in, do not touch |

### The one entry you have to add yourself

The Pinpoint moved off bus 1 and the new bus number is not something that can be worked out from
here — it is whichever I2C port the cable is physically in. The Control Hub's I2C ports are
**labelled 0, 1, 2, 3** on the case. Read the label, then:

> Configure Robot → Control Hub → **I2C Bus \<that number\>** → *goBILDA Pinpoint Odometry
> Computer* → name it exactly **`odomcomp`**

It is left out of the pushed file deliberately rather than guessed, because a wrong bus number does
not produce an error in the config — it just fails to find the device at init, which sends you
looking at the cable.

**It must not be bus 0.** The built-in IMU lives there, and sharing bus 0 causes intermittent
dropped reads that look like random odometry glitches rather than a wiring problem.

If you get it wrong, `T1 Odometry Test` tells you immediately — it prints the Pinpoint's own status
line, and a missing device says so in plain words.

## Two things to be aware of now the drivetrain is split across hubs

### 1. Re-check your motor directions first thing

This matters more than usual. Motor direction depends on how the motor is mounted **and** which way
its two power leads are connected. A rewire is exactly the moment that changes, and the symptom is a
robot that drives sideways when you push forward, or spins when you expect it to drive.

Run **`MecanumTeleOp + Odometry`** and check, in this order:

1. Push the left stick **forward** — the robot should go forward. If a wheel fights, flip that
   corner's flag in `RobotConstants.Drive` (`REVERSE_FRONT_LEFT`, etc.) and redeploy.
2. Push the left stick **sideways** — it should strafe the same way the stick points.
3. Push the **right** stick sideways — it should rotate.

The telemetry shows all four `pwr` values, so you can see what the code is asking for versus what the
robot does.

> The earlier "mirrored mecanum rollers" theory for the no-rotation fault may no longer apply, or may
> have changed. Redo the strafe-direction check from scratch: if the robot strafes the **opposite**
> way to the stick while forward is correct, the rollers are mirrored.

### 2. The left and right sides now live on different hubs

Left side on the Expansion Hub, right side on the Control Hub. This is a normal and legal thing to
do, and plenty of teams run it — but it is worth knowing that Expansion Hub commands travel over
RS485 rather than being set directly, which adds a small amount of latency.

Because the split is exactly left-versus-right, any such latency is asymmetric, and in principle that
shows up as a very slight veer when accelerating hard in a straight line. In practice it is usually
well under what wheel scrub and battery sag already cause, so do not go hunting for it — just be
aware of it if you later find a persistent drift that odometry tuning cannot explain.

## How to apply the configuration

### Option A — push the file (fast, no typing on a phone)

```cmd
cd /d C:\Users\MUTTA\OneDrive\Desktop\FTC-2026

:: back up what is on the Hub right now, in case you want to go back
adb pull /sdcard/FIRST/firstconfig.xml hub-backup\firstconfig-before-rewire.xml

:: push the new one
adb push hub-backup\firstconfig-rewired.xml /sdcard/FIRST/firstconfig.xml

:: restart the Robot Controller so it re-reads the file
adb shell am force-stop com.qualcomm.ftcrobotcontroller
adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
```

Then on the Driver Station, open **Configure Robot** and confirm the active configuration shows the
new ports. If it still shows the old ones, select the configuration again to force a reload.

### Option B — type it in on the Driver Station

Slower but it is the officially supported path, and it is what you will do at a competition when no
laptop is allowed on the network.

Three dots → **Configure Robot** → pick the active config → **Edit**, then set each port per the
table above. Remember:

- Motor type must be **goBILDA 5202/5203 series**, not plain "Motor"
- Servos must be **Continuous Rotation Servo**, not plain "Servo"
- **Save**, and make sure it is still the **active** configuration afterwards
