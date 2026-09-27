# Command Prompt quickstart — beginner version

Every command here is for **Command Prompt** (`cmd.exe`), not PowerShell. If you have been using the
terminal inside VS Code, that is probably PowerShell and some of these will look slightly different.
Use a real Command Prompt window and you can copy-paste everything exactly.

---

## Words you will see, in plain language

| Word | What it actually means |
|---|---|
| **Command Prompt** | A window where you type commands instead of clicking buttons |
| **Gradle** | The tool that turns your Java files into an app. `gradlew` is the copy that lives in this project |
| **APK** | The finished app file. Your robot code becomes one of these |
| **compile / build** | Turn your Java text files into an app |
| **deploy / install** | Copy that app onto the robot |
| **adb** | "Android Debug Bridge" — the program that talks to the Control Hub over USB, because the Hub is an Android device |
| **Control Hub** | The black box on the robot that runs your code |
| **Driver Station** | The phone or tablet the drivers hold |
| **OpMode** | One robot program. They show up in a list on the Driver Station |
| **IntelliSense** | The editor's autocomplete and red-underline-mistakes feature |
| **hardware map** | The list of motors and sensors you type into the Driver Station, so code can find them by name |

---

## Open a Command Prompt in the right folder

1. Press **Windows key + R**
2. Type `cmd`
3. Press **Enter**

A black window opens. Now move it into your project folder. **Copy-paste this exactly:**

```cmd
cd /d C:\Users\MUTTA\OneDrive\Desktop\FTC-2026
```

`cd` means "change directory" — go to a folder. The `/d` part is needed in case the folder is on a
different drive letter than the one you started on.

You will know it worked because the text at the left of the line changes to show that folder.

> **Tip:** you can also right-click the `FTC-2026` folder in File Explorer and choose *Open in
> Terminal*, then type `cmd` and Enter to switch to Command Prompt.

**Every command below assumes you are in this folder.** If you close the window, run the `cd /d`
command again.

---

## Step 1 — Make the editor stop showing fake errors

The editor needs a list of all the code libraries your project uses. This command builds that list.

```cmd
gradlew :TeamCode:ideLibs
```

Wait for it. When it finishes you should see something like:

```
ideLibs: staged 8 jar(s) and 34 aar(s) into ...\.ide-libs
BUILD SUCCESSFUL
```

Now tell the editor to start fresh and read that list:

1. Click on the VS Code (or Cursor) window
2. Press **Ctrl + Shift + P** — a search box appears at the top
3. Type `Developer: Reload Window`
4. Press **Enter**

The window blinks and reloads. **Wait about a minute** — it is reading 43 library files and it is slow
the first time. The bottom status bar shows what it is doing.

**How to check it worked:** open `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\RobotConstants.java`.
The yellow warning saying *"not on the classpath of project TeamCode"* should be gone. Hover your
mouse over a word like `DcMotorEx` and you should get a little description popup.

---

## Step 2 — Build the app

```cmd
gradlew :TeamCode:assembleDebug
```

This turns your Java files into an app. First time it took about 2 minutes; after that it is usually
a few seconds because it only redoes what changed.

**Success looks like:**

```
BUILD SUCCESSFUL in 3s
```

**You will also see some yellow warnings** that say `source value 8 is obsolete`. **Ignore them.**
They are not your fault and not a problem — the project has to target an old Java version on purpose,
because the browser-based OnBot Java editor only understands that version.

**If it says `BUILD FAILED`**, scroll up and look for a line with `error:` in it. It will name the
file and the line number. That is a real mistake in the code.

---

## Step 3 — Put the code on the robot

### First, plug in

1. **Turn the robot on.** The Control Hub needs its **12 V battery** connected and the switch on. It
   does **not** run off the USB cable — a common first mistake.
2. Wait a few seconds for the lights to settle.
3. **Plug a USB-C cable** from your laptop into the Control Hub's **USB-C port**.

   ⚠️ The Control Hub has three USB sockets. You want the **USB-C** one (the small oval). The two
   rectangular **Type-A** sockets are for plugging things *into* the robot, like your Limelight
   camera. Using a Type-A socket will not work.

   ⚠️ Many USB-C cables are **charge-only** and carry no data. If nothing works, try a different
   cable before assuming anything else is broken. This wastes a lot of people's time.

### Check the laptop can see the robot

`adb` is not on your system path, so type its full location. Copy-paste this whole line:

```cmd
"C:\Users\MUTTA\AppData\Local\Android\Sdk\platform-tools\adb.exe" devices
```

**Good result** — a line with a jumble of letters and numbers, then the word `device`:

```
List of devices attached
1A2B3C4D5E      device
```

**Bad result** — nothing under `List of devices attached`. See the table at the bottom.

### Optional: make `adb` shorter to type

For the current window only, this lets you type just `adb`:

```cmd
set PATH=%PATH%;C:\Users\MUTTA\AppData\Local\Android\Sdk\platform-tools
```

Then `adb devices` works. This lasts until you close the window, which is deliberately safe.

> **Do not use `setx PATH "%PATH%;..."`** to make it permanent. It looks like the obvious next step
> and it is a real trap: it squashes your system and user paths together into one, and it silently
> cuts everything off after 1024 characters. People have broken their Windows install this way. If
> you want it permanent, use **Settings → search "environment variables" → Edit user variables** and
> add the folder as a new line in `Path`.

### Send the code

```cmd
gradlew :TeamCode:installDebug
```

This builds **and** copies the app onto the robot. Takes 30–60 seconds.

**What you will see on the robot:** the Robot Controller app restarts by itself. The Driver Station
will say it lost connection for a moment, then reconnect. That is normal and means it worked.

**Success looks like:**

```
BUILD SUCCESSFUL
```

Now on the Driver Station, the OpMode dropdown has your programs in it.

### Later: do it over Wi-Fi with no cable

Nice once the robot is built and the USB port is hard to reach. Set it up **while still plugged in**:

```cmd
adb tcpip 5555
```

Now unplug the cable. On your laptop, join the robot's Wi-Fi network (it is named after your team
number). Then:

```cmd
adb connect 192.168.43.1:5555
adb devices
gradlew :TeamCode:installDebug
```

⚠️ **At a competition, unplug and disconnect before you get in the queue.** The rules (R704.C) say
programming laptops must be off the robot's network during a match, and inspectors check.

### If `adb devices` shows nothing

| What you see | What to do |
|---|---|
| Empty list | You used a Type-A socket. Use the **USB-C** one |
| Empty list | Robot not powered. Connect the 12 V battery and flip the switch |
| Empty list | Charge-only cable. Try another cable |
| `unauthorized` | Run `adb kill-server`, then `adb start-server`, then unplug and replug |
| `offline` | Unplug, wait 5 seconds, plug back in |
| Still nothing | `adb kill-server`, turn the robot off and on, plug in, try again |

---

### `INSTALL_FAILED_UPDATE_INCOMPATIBLE` — the one-time hurdle everyone hits

If the deploy gets as far as saying

```
Installing APK 'TeamCode-debug.apk' on 'Control Hub v1.0 - 7.1.2'
```

and then fails with

```
INSTALL_FAILED_UPDATE_INCOMPATIBLE: Package com.qualcomm.ftcrobotcontroller
signatures do not match the previously installed version; ignoring!
```

**Good news first: your cable, your Hub and your code are all fine.** It reached the robot and tried
to install. This is not a bug.

**Why it happens.** Every Android app is digitally signed, and Android refuses to *update* an app if
the new signature differs from the installed one — that is a security feature stopping someone
replacing an app with a lookalike. Your Control Hub came with the Robot Controller app already
installed, signed by REV/FIRST with their private key. Your build is signed with the automatic debug
key on your laptop. Your project uses the **same package name** (`com.qualcomm.ftcrobotcontroller`,
in `build.common.gradle`) because it *is* the Robot Controller app — so Android sees an update with
the wrong key and refuses.

**The fix is to remove the factory app first.** Every team that moves from OnBot Java to a real
project does this exactly once.

#### Step 1 — back up what is on the Hub (2 minutes, do not skip)

Your OnBot Java programs and your saved robot configurations live in the Hub's storage, separate from
the app, so they *should* survive. Back them up anyway — it costs nothing:

```cmd
adb pull /sdcard/FIRST C:\Users\MUTTA\Desktop\hub-backup
```

That copies your OnBot Java source files **and** your saved robot configurations onto your Desktop.

> Your `MecanumTeleOp` and `IntakeFlywheelTest` are already safe in this repository regardless —
> they were committed as `MecanumTeleOp.java` and `IntakeFlywheelTest.java`. So even a worst case
> loses nothing.

#### Step 2 — remove the factory app

```cmd
adb uninstall com.qualcomm.ftcrobotcontroller
```

You should see `Success`. The Driver Station will lose connection — expected, there is no Robot
Controller app running for a moment.

#### Step 3 — install yours

```cmd
gradlew :TeamCode:installDebug
```

Now it works, and **you never have to do this again** — from here on every deploy is just this one
command, because your builds all share the same debug key.

#### What this does and does not change

| | |
|---|---|
| The Robot Controller app | Replaced with your build. This is the point |
| Your OnBot Java source | Survives (it is in `/sdcard/FIRST/java/`, not in the app) |
| Your robot configurations | Survive (they are in `/sdcard/FIRST/settings/`) |
| The Hub's operating system | Untouched. You are only replacing one app |
| Going back | Possible any time — the official APK is on the [SDK releases page](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) |

#### One thing to check afterwards

Your **Driver Station app version must match** the Robot Controller version. You are now running
SDK **12.0**, so the Driver Station needs to be 12.0 too. If it complains about a version mismatch
after this, update the Driver Station app from the Play Store, or sideload the same-version DS APK
from that releases page.

---

## Step 4 — Tell the robot what is plugged into it

The code looks for motors **by name**. You type those names into the Driver Station. If a name does
not match, the program crashes the moment you press INIT with a message saying the device is missing.

On the **Driver Station**:

1. Tap the **three dots** in the top-right corner
2. Tap **Configure Robot**
3. Tap **New** (or pick an existing one and tap **Edit**)
4. Tap **Control Hub Portal** to open it up

You will see **Control Hub**. If your Expansion Hub does not appear underneath it, tap **Scan**.

### Control Hub → Motors

Tap **Motors**. Set each port:

| Port | Type to choose | Name to type |
|---|---|---|
| 0 | goBILDA 5202/5203 series | `leftfront` |
| 1 | goBILDA 5202/5203 series | `rightfront` |
| 2 | goBILDA 5202/5203 series | `leftback` |
| 3 | goBILDA 5202/5203 series | `rightback` |

⚠️ **Choose the goBILDA type, not plain "Motor".** Plain "Motor" has the wrong numbers built into it
for how many encoder ticks make one wheel turn, which quietly breaks speed control later. There is no
"5203" option — "5202/5203 series" is the correct one for all of your motors.

### Control Hub → I2C

Go back and tap **I2C Bus 1**.

| Type to choose | Name to type |
|---|---|
| goBILDA Pinpoint Odometry Computer | `pinpoint` |

⚠️ **Use Bus 1, 2, or 3 — never Bus 0.** Bus 0 is already used by a sensor inside the Control Hub,
and sharing it causes random glitches that look like broken code.

⚠️ Also check the physical mounting: the Pinpoint must sit **sticker side up**. Upside down makes it
report turning the wrong way, and that is impossible to fix in software.

### Expansion Hub → Motors

| Port | Type to choose | Name to type |
|---|---|---|
| 0 | goBILDA 5202/5203 series | `intakemain` |
| 1 | goBILDA 5202/5203 series | `Flywheel` |

⚠️ **`Flywheel` starts with a CAPITAL F.** Names are case-sensitive — `flywheel` and `Flywheel` are
different names as far as the code is concerned. Your existing code uses the capital, so keep it.

### Expansion Hub → Servos

| Port | Type to choose | Name to type |
|---|---|---|
| 0 | **Continuous Rotation Servo** | `leftintake` |
| 1 | **Continuous Rotation Servo** | `rightintake` |

⚠️ Pick **Continuous Rotation Servo**, not plain "Servo". A plain servo turns to an angle and stops; a
continuous one spins like a motor, which is what an intake needs.

### The Limelight camera

This one is not in the dropdown list — the robot finds it by itself.

1. Plug the Limelight into the Control Hub's **blue USB 3.0** rectangular socket, using a
   USB-C-to-USB-A cable. It needs no separate power.
2. In the configuration screen, tap **Scan**.
3. A new device appears called **`Ethernet Device`**. Rename it to `limelight`.
4. **Leave the IP address box alone.** The default is right.

> There are two different IP addresses involved and mixing them up is the classic mistake. The small
> grey text under the device name is the address the *camera gave the Control Hub*. The box you can
> edit wants the *camera's own* address. Don't change it.

### Save it — and don't forget this part

1. Tap **Save**
2. Name it something like `BioBuzz`
3. Go back and make sure it is the **active** configuration — the Driver Station shows the active
   one at the top

⚠️ Saving a configuration does **not** switch to it. Forgetting to activate it is one of the most
common half-hour time-wasters in FTC.

### What you can leave out for now

`turret`, `feeder`, `ball_color`, `Webcam 2` — these are in the code for parts you have not built
yet. Only the `TeleOp Main` program needs them, and it is turned off. Everything below works without
them.

---

## Step 5 — Find out why the robot will not rotate

On the Driver Station, pick **`MecanumTeleOp + Odometry`**, tap **INIT**, then **START**.

**Important thing to know first:** your code *already has* rotation in it. The line
`rx = gamepad1.right_stick_x` is the rotation input and it is mixed into all four wheels correctly.
So we are not adding anything — we are finding what is blocking it.

### Test 1 — is the controller sending anything?

Look at the line on screen that says **`right_stick_x`**. Now wiggle the **right** stick left and
right.

**The number changes** → the controller is fine. Go to Test 2.

**The number stays at 0.000** → the problem is the controller, not the robot:

- You are using a **PlayStation controller** (your code uses `cross` and `triangle`, which are PS
  buttons). Look at the little **gamepad icon on the Driver Station — it must not be orange.** Orange
  means the controller connected through a backup driver, and when that happens the sticks get mapped
  to the wrong places.
- Make sure the controller is registered as **driver 1**: press **start + cross** together.
- Try the built-in **`Test Gamepad`** program (in the Driver Station's Utility menu) to see the raw
  sticks without our code involved.
- If it is a **Logitech F310**: there is a small switch on the back that says **X** and **D**. It must
  be on **X**. The robot only reads that switch when you plug the controller in, so after flipping it
  you must **unplug and replug** the controller.

### Test 2 — hold the Y button

Y makes the robot spin using a fixed value, completely ignoring the sticks.

**It spins when you hold Y, but not with the stick** → the robot is fine, the stick value is not
getting through. Go back to Test 1.

**It does not spin with Y either** → the problem is physical. Look at the four `pwr` numbers on
screen; they should read about `+0.3, +0.3, -0.3, -0.3`. If those numbers look right but the robot
still does not turn, go to Test 3.

### Test 3 — look at your wheels (this is the most likely answer)

**Stand over the robot and look down at it.** Mecanum wheels have angled rollers on them. From above,
those rollers must form an **X** shape across the robot.

If they are installed mirrored — making a diamond shape, or all pointing the same way — here is
exactly what happens, and it matches your symptoms perfectly:

- Driving forward: **works perfectly, totally unaffected**
- Strafing sideways: **works, but goes the wrong direction**
- Turning: **almost completely cancels itself out** — you keep about 7% of normal turning force, and
  on a square robot you get *exactly zero*

**The 10-second test:** push the left stick straight sideways. If the robot slides the **opposite way**
from the stick, while driving forward is correct, then your rollers are mirrored — and that by itself
explains why it will not turn.

### Test 4 — other physical things

- Lift the robot up and spin each wheel by hand. All four should turn freely.
- Put it down and make sure **all four wheels touch the floor**. Turning needs all four; driving
  straight does not. So a slightly twisted frame breaks turning first.
- Check the little set screws on the wheel hubs. A loose one means the motor spins but the wheel
  does not.

### Things you can stop worrying about

- **Motor directions being flipped on the wrong side.** That would break forward, sideways *and*
  turning all at once. Since forward works, this is not it.
- **One dead motor.** That does not stop turning either. The fact that you drive straight cleanly
  actually *proves* your wiring is fine.
- **The maths in the code.** Do not let anyone "fix" the plus and minus signs. They match the official
  FTC example exactly. Changing them will break driving forward, which currently works.

---

## The five commands, all in one place

```cmd
cd /d C:\Users\MUTTA\OneDrive\Desktop\FTC-2026

gradlew :TeamCode:ideLibs              :: refresh editor, then Ctrl+Shift+P > Reload Window
gradlew :TeamCode:assembleDebug        :: build the app
gradlew :TeamCode:installDebug         :: build AND send it to the robot
gradlew :TeamCode:tasks                :: list everything you can run
```

`::` starts a comment in Command Prompt — you do not need to type those parts.
