# Start here — FTC BIOBUZZ 2026-27 setup and odometry bring-up

## 0. Two things about your tools, before anything else

**The WPILib extension does nothing for you. Uninstall it.** WPILib is the *FRC* (FIRST Robotics
Competition) library — different program, different control system, different language stack. FTC
does not use it at any point. It will only confuse the workspace.

**There is no official FTC extension for VS Code.** FIRST officially supports exactly three ways to
program an FTC robot:

| Tool | What it is | Good for |
|---|---|---|
| **Android Studio** | Full IDE, builds this Gradle project | Serious season-long code. This repo. |
| **OnBot Java** | Browser editor served by the Control Hub | Quick fixes at competition, no laptop setup |
| **Blocks** | Browser drag-and-drop | Rookies, first week |

The "FTC for VS Code" extension you may have installed is a community project by a team
(Juice16236) with, in its own words, *no official affiliation with FTC or FIRST*. It is a
convenience wrapper around snippets and `adb` — it does not build anything by itself, and it still
requires you to install the Android SDK and a JDK manually.

### So what should you actually do?

**VS Code / Cursor works, and it is already set up and verified on this machine.** No Android Studio
required. The toolchain was installed and the project builds:

```
BUILD SUCCESSFUL
TeamCode/build/outputs/apk/debug/TeamCode-debug.apk   (52.5 MB)
```

What is installed and working:

| Piece | Version | Where |
|---|---|---|
| Android SDK command-line tools | rev 23 | `%LOCALAPPDATA%\Android\Sdk\cmdline-tools\latest` |
| Android platform | android-30 | matches `compileSdkVersion 30` |
| Build tools | 35.0.0 | required by AGP 8.13.2 |
| platform-tools (`adb`) | 37.0.1 | `%LOCALAPPDATA%\Android\Sdk\platform-tools` |
| JDK | Temurin **25** | already on your PATH — it works, see below |
| Gradle | 9.1.0 | downloaded by the wrapper |

Everyday commands:

```bash
./gradlew :TeamCode:assembleDebug     # compile
./gradlew :TeamCode:installDebug      # deploy to the robot over USB
./gradlew :TeamCode:ideLibs           # refresh editor IntelliSense (see below)
```

**Android Studio is still a reasonable fallback** if something in the editor tooling misbehaves — it
handles all of this natively — but you do not need it, and nothing here depends on it.

> **JDK 25 works. Settled by building with it, not by reading docs.** An earlier version of this
> guide said JDK 25 "will likely fail a command-line build". It doesn't: a full
> `:TeamCode:assembleDebug` — javac, D8/R8, AAPT2, packaging, the whole chain — completes on
> Temurin 25 and produces a working APK.
>
> For the record: Gradle's compatibility matrix lists Java 25 as supported for running Gradle from
> **9.1.0** onward, and this project's wrapper is pinned to exactly `gradle-9.1.0-bin`. AGP 8.13
> documents a JDK *minimum* of 17 and states no maximum. You will see a deprecation warning about
> `source`/`target` value 8 being obsolete — that comes from the project targeting Java 8 (which it
> must, because OnBot Java only supports Java 8), not from your JDK. It is noise.
>
> If you ever want every machine pinned to one JDK, the portable way is **not**
> `org.gradle.java.home` (an absolute path that cannot be committed). Run
> `./gradlew updateDaemonJvm --jvm-version=21` once, which writes
> `gradle/gradle-daemon-jvm.properties`, and commit that — then CLI and IDE agree on every computer.

Download: <https://developer.android.com/studio>

**You need Android Studio "Narwhal 3 Feature Drop" or newer.** Older versions cannot sync AGP
8.13.2 — and if Android Studio offers to *downgrade* the AGP for you, **say no**. Gradle 9.1 removed
features the older AGP needs, so downgrading breaks the build in a more confusing way.

<details>
<summary>Setting this up on ANOTHER computer — the exact steps that worked here</summary>

1. **JDK.** Temurin 21 or 25 both work. <https://adoptium.net/>
2. **Android SDK command-line tools** — the "command line tools only" download:
   <https://developer.android.com/studio#command-line-tools-only>
   Extract so that `sdkmanager`/`android` ends up at
   `<SDK>\cmdline-tools\latest\bin\`. The zip has a top-level `cmdline-tools` folder you rename to
   `latest`.
3. **Install the packages.** Note two things that cost time here:
   - `sdkmanager` is **deprecated** in cmdline-tools rev 23. It still works but delegates to a new
     `android` CLI whose syntax is `android sdk install <package>`.
   - Call `android.exe` **directly** and quote the package name. Going through `sdkmanager.bat`
     splits `platforms;android-30` on the semicolon and you get
     *"Package platforms not found. Package android-30 not found."*
   ```bash
   android.exe sdk install "platform-tools"
   android.exe sdk install "platforms;android-30"
   android.exe sdk install "build-tools;35.0.0"
   ```
   The new CLI exits with a stack-buffer-overrun code **after** finishing successfully. Ignore the
   exit code and check the files instead.
4. **Create `local.properties`** in the repo root with that machine's path:
   ```properties
   sdk.dir=C\:\\Users\\<you>\\AppData\\Local\\Android\\Sdk
   ```
   It is gitignored, so every machine needs its own.
5. **Build**: `./gradlew :TeamCode:assembleDebug`
6. **IntelliSense**: `./gradlew :TeamCode:ideLibs`, then *Developer: Reload Window*.

**Do NOT let AGP install the SDK packages for you.** It tries, and on this setup it hangs: rev 23
cmdline-tools writes SDK metadata at XML version 4 while AGP 8.13 only understands up to version 3.
You will see `This version only understands SDK XML versions up to 3 but an SDK XML file of version 4
was encountered`, followed by the build sitting forever on
`Preparing "Install Android SDK Build-Tools 35"`. Pre-installing the packages in step 3 avoids it
entirely. If you interrupt AGP mid-install, delete the partial `build-tools\35.0.0` folder and
reinstall — a half-installed one is missing `aapt2.exe` and fails later with a confusing error.

**Which versions you actually need, and where each comes from:**
- `compileSdkVersion 30` → `platforms;android-30`, from `build.common.gradle`
- `build-tools;35.0.0` → **from AGP, not the project.** It is newer than the compile SDK, which
  looks wrong and isn't.
- The `ndkVersion 21.3.6528147` in `build.common.gradle` is **not needed** — this project builds
  without it. Don't spend the gigabyte.
</details>

### Editor IntelliSense

VS Code showed *"not on the classpath of project TeamCode, only syntax errors are reported"*. That
has a real root cause: **the Java extension cannot import an Android Gradle project.** It understands
plain Java/Gradle, not the Android Gradle Plugin, so the import was never going to succeed and no
amount of *Clean Java Language Server Workspace* fixes it.

The fix, already committed in [`.vscode/settings.json`](../.vscode/settings.json): turn the Gradle
importer off and describe the project manually as an unmanaged folder. Dependency jars are staged
into `.ide-libs/` by a Gradle task, because the extension will not resolve paths into the Gradle
cache in your home directory.

```bash
./gradlew :TeamCode:ideLibs      # stages 43 jars incl. android.jar
```
then **Developer: Reload Window**. Re-run it whenever dependencies change.

Install these two extensions (both on Open VSX, so Cursor can too):
`vscjava.vscode-java-pack` and `vscjava.vscode-gradle`. They are recommended automatically via
[`.vscode/extensions.json`](../.vscode/extensions.json).

---

## 1. A warning about this folder's location

This repo lives in `OneDrive\Desktop\FTC-2026`. OneDrive continuously syncs it, and Gradle writes
thousands of files into `build/` directories during every compile. That combination causes real
problems: file-lock errors mid-build, sync conflict copies like `RobotConstants-DESKTOP-ABC.java`
appearing in your source tree, and slow builds.

Pick one of these:

- **Best:** move the project out of OneDrive, e.g. to `C:\FTC\FTC-2026`. Your code is safe in git
  instead — see §6.
- **Acceptable:** right-click the OneDrive tray icon → Settings → Sync and backup → Advanced
  settings, and exclude `build`, `.gradle`, and `.idea`.

The `.gitignore` that ships with the SDK already excludes those from git, so nothing is lost either
way.

---

## 2. What is already done

I have set this folder up as a git repository with the official SDK in it:

```
FTC-2026/
├── FtcRobotController/          <- the SDK app. DO NOT EDIT. Pull updates into it.
├── TeamCode/                    <- your code. Everything you write goes here.
│   └── src/main/java/org/firstinspires/ftc/teamcode/
│       ├── RobotConstants.java          every tunable number, one file
│       ├── AimController.java           ties odometry + vision + ballistics together
│       ├── subsystems/
│       │   ├── Odometry.java            goBILDA Pinpoint wrapper  <- START HERE
│       │   ├── MecanumDrive.java        robot- and field-centric drive
│       │   ├── Turret.java              yaw axis with rotation feedforward
│       │   ├── Shooter.java             flywheel, closed-loop RPM
│       │   └── BallSorter.java          colour sensor -> POLLEN / NECTAR
│       ├── vision/
│       │   ├── HiveTracker.java         AprilTag cluster -> field-frame target
│       │   └── BallTracker.java         finds balls on the floor
│       ├── util/
│       │   ├── Ballistics.java          the shoot-while-moving solver
│       │   └── BallType.java            ball specs + per-type shooter calibration
│       └── opmodes/
│           ├── TeleOpDriveOnly.java     <- run this one first
│           ├── TeleOpMain.java          full robot (@Disabled until hardware exists)
│           └── tuning/T1..T6            the tuning sequence, in order
└── docs/                        <- these guides
```

- **SDK version 12.0** (`20260907-090034`), the BIOBUZZ release, checked out at tag `v12.0`.
- Remote `upstream` points at FIRST's repo, so you can pull mid-season SDK updates.

> **Note:** I could not compile-verify this code, because there is no Android SDK on this machine
> yet. The APIs used were all read directly out of the SDK source now sitting in
> `FtcRobotController/`, and one field I could not find there (`frameAcquisitionNanoTime`) I
> confirmed against the Vision 12.0.0 Javadoc. Expect it to build, but your first Gradle sync is the
> real test — see §4.

---

## 3. Open the project

1. Launch Android Studio → **Open** → select `C:\Users\MUTTA\OneDrive\Desktop\FTC-2026`.
2. Wait for **Gradle sync**. First run downloads Gradle 9.1 and the FTC libraries; expect 5–15
   minutes on a slow connection. Watch the bottom status bar.
3. When it finishes, the dropdown at the top should offer a **TeamCode** run configuration.

If sync fails, read the actual error before changing anything. The two common ones:
- *"Android Gradle plugin requires a newer Android Studio"* → your Android Studio is too old. Update
  it. Do not downgrade AGP.
- *"SDK location not found"* → Android Studio will offer to install the SDK; accept.

---

## 4. Sanity-check the build before you touch hardware

```bash
./gradlew :TeamCode:assembleDebug
```

This compiles without needing a robot. If anything I wrote has a typo, it surfaces here — and the
error will name the file and line. Fix or tell me, then continue.

---

## 5. Set up the robot itself

### 5a. Wire the odometry

Your 4-Bar Odometry Pack is two pods and one Pinpoint computer.

1. Mount the Pinpoint **sticker side up**. Upside down inverts the IMU heading, and you will chase
   that as a software bug for an hour.
2. Plug it into **I2C bus 1, 2 or 3 on the Control Hub — not bus 0.** Bus 0 is shared with the
   Control Hub's built-in IMU; the contention shows up as intermittent bad reads.
3. Pods: the **X pod points forward** (it rolls when the robot drives forward) and the **Y pod points
   sideways** (it rolls when the robot strafes). Both pod wheels must be pressed firmly to the floor.
4. Measure the two offsets, in millimetres, from your robot's **centre of rotation** to each pod:
   - `X_POD_OFFSET_MM` — how far *sideways* the forward-facing pod sits. **Left is positive.**
   - `Y_POD_OFFSET_MM` — how far *forward* the sideways-facing pod sits. **Forward is positive.**

   Put them in `RobotConstants.Odo`.

### 5b. Robot Configuration on the Driver Station

Create a configuration and add:

| Device | Port | Name — must match exactly |
|---|---|---|
| goBILDA Pinpoint Odometry Computer | I2C bus 1 | `pinpoint` |
| Motor (drive) | 0 | `front_left` |
| Motor (drive) | 1 | `front_right` |
| Motor (drive) | 2 | `back_left` |
| Motor (drive) | 3 | `back_right` |
| Webcam | USB | `Webcam 1` |

Add the shooter/turret/intake/colour-sensor names from `RobotConstants.Names` later, as you build
them. **A name mismatch is the single most common cause of "it crashes on init".**

> As of SDK 12.0 the Pinpoint driver ships *inside the SDK* as
> `com.qualcomm.hardware.gobilda.GoBildaPinpointDriver`. Older guides tell you to copy
> `GoBildaPinpointDriver.java` into TeamCode from goBILDA's GitHub — **do not do that now.** You will
> end up with two copies of the class and a confusing compile error.

### 5c. Deploy

Connect the Control Hub by USB (or join its Wi-Fi), pick **TeamCode** in the dropdown, press Run.
The OpModes appear on the Driver Station under the groups `Tuning` and `Drive`.

---

## 6. Put it on GitHub

Nothing is pushed anywhere yet. From the repo root:

```bash
# On github.com: create a new EMPTY repo, e.g. FTC-BioBuzz-2026. Do not add a README.
git remote add origin https://github.com/<your-team>/FTC-BioBuzz-2026.git
git push -u origin main
```

`upstream` is already FIRST's repo, so when FIRST releases SDK 12.1 mid-season:

```bash
git fetch upstream --tags
git merge v12.1        # your TeamCode/ is untouched; only FtcRobotController/ changes
```

That separation is exactly why you never edit `FtcRobotController/`.

---

## 7. The tuning sequence — do these in order

Each OpMode explains itself on the Driver Station telemetry, and the Javadoc at the top of each file
explains *why*. **Do not skip ahead.** Every step depends on the one before it being right, and
debugging a turret on top of a reversed encoder is a bad afternoon.

| # | OpMode | What it settles | Needs |
|---|---|---|---|
| 1 | `T1 Odometry Test` | Pinpoint is healthy; encoder directions; distance accuracy | Pinpoint only — push the robot by hand |
| 2 | `T2 Pinpoint Offset Tuner` | Pod offsets, so rotating in place doesn't drift the pose | Pinpoint only |
| 3 | `T3 Velocity Frame Test` | Whether Pinpoint velocity is field- or robot-relative | Pinpoint only |
| — | `TeleOp Drive Only` | Odometry survives real driving | + 4 drive motors |
| 4 | `T4 Hive Tracker Test` | Camera mount numbers; **the BIOBUZZ cluster names** | + webcam |
| 5 | `T5 Shooter RPM Tuner` | `transferEfficiency` per ball type | + flywheel |
| 6 | `T6 Color Sensor Calibration` | Hue windows for POLLEN / RED / BLUE | + colour sensor |
| — | `TeleOp Main` | Everything together | full robot |

**Today, with just the odometry pack, you can do steps 1–3.** That is a genuinely productive first
session and it is the right place to start.

### Why step 3 exists and is not optional

goBILDA's driver documents the velocity getters as *"X (forward) velocity"* — which sounds
robot-relative — but it documents *position* with the same wording, and position is unambiguously a
field coordinate. The labels come from the pod names and are not evidence either way. Rather than
guess, `T3` measures it on your robot in twenty seconds.

Get it wrong and nothing crashes. The turret simply leads correctly when you drive along one axis
and leads *backwards* when you drive along another. That is close to impossible to diagnose from the
driver's seat during a match.

### A note on step 2

The usual folk wisdom — "spin the robot ten full turns and see if the pose returns to zero" — does
not work for pod offsets. Work out the drift an offset error produces over a rotation of angle `T`
and you get

```
field error = ( -dy*(1 - cos T) - dx*sin T ,  -dx*(1 - cos T) + dy*sin T )
```

At `T = 360°` both terms vanish: **a full turn hides the error completely.** At `T = 180°` it becomes
a clean `reportedX = -2*dy`, `reportedY = -2*dx`. So `T2` has you do one accurate *half* turn and
reads both corrections straight off. The residual is periodic, so spinning further gains nothing —
what matters is landing exactly on 180° with the tracking point over one spot.

---

## 8. Where to read next

- [`01-BIOBUZZ-GAME.md`](01-BIOBUZZ-GAME.md) — the game, the ball specs, scoring, official links
- [`02-APRILTAGS-AND-VISION.md`](02-APRILTAGS-AND-VISION.md) — the tags ("QR codes"), where to
  download and print them, and an important correction about training a CV model
- [`03-ARCHITECTURE.md`](03-ARCHITECTURE.md) — how the auto-aiming turret works, and the one design
  assumption in your plan that BIOBUZZ breaks

---

## Useful references

- FTC SDK: <https://github.com/FIRST-Tech-Challenge/FtcRobotController>
- FTC docs: <https://ftc-docs.firstinspires.org>
- SDK Javadoc: <https://javadoc.io/doc/org.firstinspires.ftc>
- Pinpoint product page + user guide: <https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/>
- FTC community forum: <https://ftc-community.firstinspires.org>
