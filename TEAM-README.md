# FTC BIOBUZZ 2026-27 — team code

Built on **FTC SDK v12.0** (`20260907-090034`), the BIOBUZZ release.

> This file, not `README.md`, is our entry point. `README.md` belongs to the SDK and carries FIRST's
> release notes — leaving it untouched means SDK updates merge cleanly instead of conflicting every
> time.

## → [Start here: `docs/00-START-HERE.md`](docs/00-START-HERE.md)

| Doc | What's in it |
|---|---|
| [`00-START-HERE.md`](docs/00-START-HERE.md) | Toolchain setup, build, deploy, robot config, **the ordered tuning sequence** |
| [`01-BIOBUZZ-GAME.md`](docs/01-BIOBUZZ-GAME.md) | The game, ball specs, scoring, official links |
| [`02-APRILTAGS-AND-VISION.md`](docs/02-APRILTAGS-AND-VISION.md) | AprilTag downloads and GitHub sources, cluster API, CV model reality check |
| [`03-ARCHITECTURE.md`](docs/03-ARCHITECTURE.md) | How the auto-aiming turret works and why it's built that way |

## Run these in order

| OpMode | Hardware needed |
|---|---|
| `T0 Tag Library Dump` | **none** — Control Hub only |
| `T1 Odometry Test` | Pinpoint |
| `T2 Pinpoint Offset Tuner` | Pinpoint |
| `T3 Velocity Frame Test` | Pinpoint |
| `TeleOp Drive Only` | + 4 drive motors |
| `T4 Hive Tracker Test` | + webcam |
| `T5 Shooter RPM Tuner` | + flywheel |
| `T6 Color Sensor Calibration` | + colour sensor |
| `TeleOp Main` | full robot |

## Rules of the road

1. **Only ever edit `TeamCode/`.** `FtcRobotController/` is FIRST's; changing it breaks SDK updates.
2. **All tunable numbers live in `RobotConstants.java`.** No magic numbers in subsystems. If you find
   yourself typing a number into a subsystem, it belongs in the constants file with a comment saying
   which OpMode measures it.
3. **One convention everywhere: +X forward, +Y left, +heading counter-clockwise, inches and radians.**
   It matches the Pinpoint and the SDK's AprilTag `bearing`, so nothing needs a sign flip.
4. **Call each sensor's `update()` exactly once per loop**, near the top, in dependency order:
   odometry → colour → vision → aim.
5. **Pull SDK updates, don't re-clone:** `git fetch upstream --tags && git merge v12.x`


## Pedro tools: AutoTune and test autos

### AutoTune
Connect to the robot's Wi-Fi and open **http://192.168.43.1:10158**. Procedures (in `pedro/Tuning.java`):

| Procedure | Use it when |
|---|---|
| Mecanum Tuner | drive motors rewired or replaced (wheels off the ground) |
| Pinpoint Tuner | pods moved. **Verify offsets** with a 180° spin test: x/y should stay within ±0.5" at the halfway point. Hand-measured offsets beat the tuner on this robot. |
| Foresight Tuner | robot weight changed noticeably (turret, shooter). Needs space ahead and to the left. |
| Tests | Driving, Localization, Hold, Line, Curve checks with current settings |

All procedures read the same settings as matches: `RobotConstants` (names, directions, pod offsets) and `FollowerTuning` (Foresight).

### Test autos (`opmodes/autos`)
| Auto | Does | Space needed |
|---|---|---|
| First Auto | 24" forward | 3 ft ahead |
| Square Auto | 24" square, ends at start | 4 ft ahead + left |
| Curve Auto | curve out and back | 4 ft ahead + left |
| Intake Curve Auto | Curve Auto + picks up a pollen | 4 ft ahead + left |

Good result for Square/Curve: ends within ~1" of the start mark, heading near 0.

### ⚠️ Pedro 3 known issue: `line()` heading is reversed
On `line(a, b)` paths, `.linear(a, b)` turns the robot the **wrong way**. Workaround:

| Path | Heading | Write |
|---|---|---|
| `line(a, b)` | headings differ | `.linear(b, a)` (swapped) |
| `curve(a, ctrl, b)` | | `.linear(a, b)` (normal) |
| either | | `.tangent()` works fine |

If a Pedro update fixes this, swapped lines (marked with comments) must be swapped back.

### Robot configuration
The active hub config is **`rewire`**. Note: `hub-backup/firstconfig-rewired.xml` is outdated (it has the left/right hubs swapped). In the real config, the **left** drive motors are on the **Control Hub** and the **right** ones on the **Expansion Hub**.