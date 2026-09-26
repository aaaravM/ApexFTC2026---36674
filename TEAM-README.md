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
