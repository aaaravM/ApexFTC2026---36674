# Deploy, configure, diagnose

Everything here has been verified on this machine except the steps that need the physical robot.

---

## 1. Reload the editor so IntelliSense works

Do this once now, and again whenever dependencies change.

```bash
cd C:\Users\MUTTA\OneDrive\Desktop\FTC-2026
./gradlew :TeamCode:ideLibs
```

Expected output:

```
ideLibs: included C:\Users\MUTTA\AppData\Local\Android\Sdk\platforms\android-30\android.jar
ideLibs: staged 8 jar(s) and 34 aar(s) into ...\.ide-libs
```

Then in VS Code / Cursor:

1. **Ctrl+Shift+P**
2. Type `Developer: Reload Window`
3. Enter

**How to know it worked:** open
[`RobotConstants.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotConstants.java)
and the warning *"not on the classpath of project TeamCode, only syntax errors are reported"* should
be gone. Hover over `DcMotorEx` anywhere and you should get a real tooltip. Ctrl-click
`GoBildaPinpointDriver` and it should jump to the decompiled class.

If it is still complaining, give the Java extension a moment — it reindexes 43 jars on first load;
the status bar shows progress. If it never settles, run `Java: Clean Java Language Server Workspace`
and reload once more.

---

## 2. Put the code on the Control Hub

### One-time: put `adb` on your PATH

```powershell
setx PATH "$env:PATH;C:\Users\MUTTA\AppData\Local\Android\Sdk\platform-tools"
```

Close and reopen the terminal. Check with `adb version` (should say 1.0.41 / 37.0.1).

You don't strictly need this — Gradle calls `adb` itself — but it makes diagnosing a connection
problem much easier.

### Deploy over USB — do it this way first

1. **Power the Control Hub on.** 12 V battery connected, switch on. Wait for the LED to settle.
2. **Plug the USB-C cable** from your laptop into the Control Hub's **USB-C port**.
   Not one of the Type-A ports — those are host ports for the Limelight and webcams. The USB-C port
   is the device port that talks to a PC.
3. **Confirm the Hub is visible:**
   ```bash
   adb devices
   ```
   You want a line with a serial and `device`. If the list is empty, see troubleshooting below.
4. **Deploy:**
   ```bash
   ./gradlew :TeamCode:installDebug
   ```
   Takes 30-60 s. The Robot Controller app restarts on its own; the Driver Station will briefly
   report losing connection and then come back.
5. On the Driver Station, the new OpModes appear in the dropdown.

### Deploy over Wi-Fi (no cable)

Convenient once the robot is assembled. Set it up while still on USB:

```bash
adb tcpip 5555
```

Unplug the cable, join the Control Hub's Wi-Fi network from your laptop, then:

```bash
adb connect 192.168.43.1:5555
adb devices                       # should now show 192.168.43.1:5555
./gradlew :TeamCode:installDebug
```

**At a competition, unplug and disconnect before you queue.** Manual rule R704.C: all programming
laptops must be off the Robot Controller network during match play. Inspectors look for this.

### If `adb devices` is empty

| Symptom | Fix |
|---|---|
| Nothing listed | Wrong port — you want the Hub's **USB-C** port, not Type-A |
| Nothing listed | Hub not powered. It does not run off USB; it needs the 12 V battery |
| `unauthorized` | `adb kill-server` then `adb start-server`, and replug |
| `offline` | Replug the cable; try a different cable — many USB-C cables are charge-only |
| Still nothing | `adb kill-server`, unplug, power-cycle the Hub, plug in, `adb devices` |

### First deploy only: `INSTALL_FAILED_UPDATE_INCOMPATIBLE`

```
INSTALL_FAILED_UPDATE_INCOMPATIBLE: Package com.qualcomm.ftcrobotcontroller
signatures do not match the previously installed version; ignoring!
```

Expected once, and it means the deploy reached the robot. `build.common.gradle` sets
`applicationId 'com.qualcomm.ftcrobotcontroller'` — the same package as the factory Robot Controller
app, because your build *is* the Robot Controller. The Hub's copy is signed with REV/FIRST's release
key and yours with your machine's debug key, and Android refuses a cross-signature update.

```cmd
adb pull /sdcard/FIRST C:\Users\MUTTA\Desktop\hub-backup   :: back up OnBot code + configs
adb uninstall com.qualcomm.ftcrobotcontroller
gradlew :TeamCode:installDebug
```

OnBot Java sources (`/sdcard/FIRST/java/src/`) and the robot configuration
(`/sdcard/FIRST/firstconfig.xml`) live
in Hub storage, not app storage, so they survive the uninstall — the backup is cheap insurance, not a
necessity. Afterwards, confirm the **Driver Station app is also 12.0**, since RC and DS versions must
match.

---

## 3. Robot Configuration, device by device

On the **Driver Station**: three-dot menu (top right) → **Configure Robot** → **New** (or select an
existing config and press **Edit**).

You will see a **Control Hub Portal**. Expand it. Under it is the **Control Hub** itself, and — since
your flywheel is on one — an **Expansion Hub**.

> If the Expansion Hub is not listed, press **Scan** with it powered and the RS485 cable connected.

### Control Hub → Motors

| Port | Name | Type |
|---|---|---|
| 0 | `leftfront` | goBILDA 5202/5203 series |
| 1 | `rightfront` | goBILDA 5202/5203 series |
| 2 | `leftback` | goBILDA 5202/5203 series |
| 3 | `rightback` | goBILDA 5202/5203 series |

**Pick the goBILDA motor type, not the generic "Motor".** The generic entry has the wrong
ticks-per-revolution and max RPM, which silently breaks `setVelocity()` scaling and the SDK's built-in
PIDF defaults. There is no "5203" entry — 5202/5203 is the right one, and it is accurate to within
0.5% for every variant you own because the encoder sits before the gearbox.

### Control Hub → I2C

| Bus | Name | Type |
|---|---|---|
| **1** | `pinpoint` | goBILDA Pinpoint Odometry Computer |

**Bus 1, 2 or 3 — never bus 0.** Bus 0 is shared with the Control Hub's internal IMU and the
contention shows up as intermittent bad reads that look like random odometry glitches.

Also, physically: the Pinpoint mounts **sticker side up**. Upside down inverts the IMU heading, and
you will chase that as a software bug.

### Expansion Hub → Motors

| Port | Name | Type |
|---|---|---|
| 0 | `intakemain` | goBILDA 5202/5203 series |
| 1 | `Flywheel` | goBILDA 5202/5203 series |

**`Flywheel` with a capital F.** `hardwareMap.get` is case-sensitive, and this is the name your
existing code already uses. Get it wrong and every OpMode throws at init with a message saying the
device is missing, which sends you looking at wiring.

### Expansion Hub → Servos

| Port | Name | Type |
|---|---|---|
| 0 | `leftintake` | **Continuous Rotation Servo** |
| 1 | `rightintake` | **Continuous Rotation Servo** |

Continuous Rotation, not Servo. A plain Servo takes a 0-1 position; `CRServo` takes a -1..1 power,
which is what the code sets.

### The Limelight

Not in the device dropdown — it is auto-discovered.

1. Plug the Limelight into the Control Hub's **USB 3.0 (blue) Type-A** port with a USB-C-to-USB-A
   cable. No separate power; it runs off the port.
2. In the configuration screen, press **Scan**.
3. A device appears named **`Ethernet Device`**. Rename it to `limelight`.
4. Leave the IP field alone.

> **Two IP addresses, and confusing them is the classic failure.** The small grey text under the
> device name is the address the Limelight handed to the *Control Hub*. The editable IP field wants
> the *Limelight's own* address. The default is correct; don't "fix" it.

### Save and activate

Press **Save**, name it something like `BioBuzz`, then make sure it is the **active** configuration
(it shows at the top of the Driver Station). A saved-but-not-activated config is a classic
half-hour loss.

### What you can skip for now

`turret`, `feeder`, `ball_color` and `Webcam 2` are in `RobotConstants` for later. Only
`TeleOp Main` needs them, and it is `@Disabled`. Everything you are about to run works without them.

---

## 4. Diagnose the rotation fault

Deploy, select **`MecanumTeleOp + Odometry`**, press INIT then START.

Your code already contains rotation — `rx = gamepad1.right_stick_x` mixed into all four wheels, which
is the standard Game Manual 0 mixing and correct for your reversed-left-side motors. So this is not
about writing code; it is about finding what stops `rx` reaching the wheels.

### Step 1 — is the gamepad giving us a value?

Watch the **`right_stick_x`** line and wiggle the right stick.

**The number moves** → gamepad is fine, go to step 2.

**The number does not move** → it is the gamepad:

- You are on a **PS4/PS5 controller** (your code uses `cross`/`triangle`/`circle`). Check the
  **gamepad indicator on the Driver Station is not orange.** SDK 12.0 colours it orange when the pad
  fell back to Android's generic driver instead of the Driver Station's own USB driver — and then the
  axis mapping is whatever the kernel decided, not what the SDK documents.
- Make sure the pad is bound as **driver 1**: press **start + a** (start + cross).
- Run the built-in **`Test Gamepad`** utility (Utility menu on the Driver Station) to see every axis
  independently of our code.
- If it is a Logitech F310: the rear **X/D switch must be on X**. It is read at USB enumeration, so
  flipping it is not enough — **unplug and replug**.

### Step 2 — hold Y

Y commands a fixed rotation directly, with no stick in the path.

**Robot rotates on Y but not on the stick** → the drivetrain and the mixing are fine. Go back to
step 1; the stick value is not arriving.

**Robot does not rotate on Y either** → the drivetrain. Check the four `pwr` values on telemetry.
They should read roughly `+0.3, +0.3, -0.3, -0.3`. If the numbers are right and the robot still will
not turn, it is mechanical → step 3.

### Step 3 — the rollers (most likely cause)

**Look down at the robot from above.** The top rollers must form an **X**.

A mirrored pattern — a diamond, or all parallel — is the one fault that reproduces your exact
symptoms. Working out the mechanics: it leaves forward drive **completely unaffected**, **silently
inverts strafe**, and collapses net rotation torque to `|b−a|/(a+b)` of normal — about **6.7% on a
14×16 in chassis, and exactly zero on a square one**.

**The 10-second test:** command a pure strafe with the left stick. If the robot strafes the
**opposite way** to the stick while forward is correct, the rollers are mirrored. That alone explains
dead rotation.

### Step 4 — mechanical

- Lift the robot and spin each wheel by hand. All four must turn freely.
- Confirm all four wheels touch the floor. Mecanum rotation needs all four loaded; translation does
  not, so a racked chassis breaks rotation first.
- Check the 8 mm REX hub set screws. A backed-out screw gives "the motor spins but the robot
  doesn't move".

### What you can rule out immediately

- **Reversing the wrong side** inverts forward, strafe **and** rotation together, so it can never
  cause a rotation-only fault.
- **A single dead motor** does not kill rotation. Clean straight translation actually *exonerates*
  your wiring.
- **The mixing maths.** Do not let anyone "fix" the signs — it matches the SDK's own
  `BasicOmniOpMode_Linear` exactly. Changing them will break forward and strafe, which currently work.

---

## 5. Then, in order

| Run | What it settles | Needs |
|---|---|---|
| `T0 Tag Library Dump` | prints the BIOBUZZ tag IDs | nothing at all |
| `T1 Odometry Test` | Pinpoint health, encoder directions, distance accuracy | Pinpoint |
| `T2 Pinpoint Offset Tuner` | pod offsets, via one accurate half-turn | Pinpoint |
| `T3 Velocity Frame Test` | field- vs robot-relative velocity | Pinpoint |
| `Intake + Flywheel Tuner (RPM)` | your real shooter RPM (hint: far below 6000) | flywheel |
| `Pedro TeleOp` | Pedro driving, incl. rotation | drivetrain + Pinpoint |

For `Pedro TeleOp`, if the robot drives but will not turn, check the four motor names in
[`PedroConstants.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/PedroConstants.java)
against the physical robot. Pedro renamed v2's `leftRearMotorName` to `backLeftName` — word order
flipped and "rear" became "back" — and two transposed motors drive almost correctly in translation
while being completely wrong in rotation.
