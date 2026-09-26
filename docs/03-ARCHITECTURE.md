# How the auto-aiming turret works

Written for: your team's programmers, and anyone on the team who has to explain this to a judge.

---

## The one thing in the original plan that has to change

Your plan was: *odometry tells us where we are, so we compute the angle to the goal and shoot.*

That is the standard FTC approach, and it works in most seasons. **It does not work in BIOBUZZ**,
because the goal moves. The HIVE is bi-stable and **TIPS** when enough elements land in the
upward-facing CELL, which physically relocates the CELL and the AprilTags on it. FIRST says so
directly in the SDK 12.0 release notes:

> *"Unfortunately, since BIOBUZZ AprilTags move, they are not suitable for absolute Field
> Localization."*

So there is no fixed field coordinate to aim at, and there is no using the tags to work out where the
robot is either. Both halves of the usual pattern are unavailable.

### The fix: swap what each sensor is responsible for

| | Usual FTC pattern | **What BIOBUZZ needs** |
|---|---|---|
| Where is the **robot**? | AprilTags (absolute) | **Odometry** (Pinpoint) |
| Where is the **target**? | hard-coded field constant | **Vision** — measured live |
| How fast are we going? | odometry | **Odometry** |

Vision answers *"where is the CELL"*. Odometry answers *"how fast am I going and which way am I
facing"*. Neither sensor does the other's job.

And this is not a downgrade. FIRST put **each cluster's origin at the centre of the CELL opening**
specifically so it can be aimed at directly — so the pose vision hands you already *is* the aim point.
You skip the whole "localise, then transform to a goal constant" chain and its accumulated error.

---

## The data flow, once per loop

```
  Odometry.update()              pose, field velocity, rotation rate
         |
  BallSorter.update()            which ball is loaded  ->  which RPM curve
         |
  HiveTracker.update()           AprilTag cluster -> target position in FIELD coords
         |                       (bridged by odometry when vision drops out)
         v
  Ballistics.solve()             turret angle + flywheel RPM + flight time
         |
         +---> Turret.setTargetFieldAngleRad()
         +---> Shooter.setTargetRpm()
         |
         v
  AimController.isReadyToFire()  four gates -> feeder
```

---

## Part 1: shooting while moving

When the ball leaves a moving robot, **it keeps the robot's velocity.** So the ball's true launch
velocity is `muzzle velocity + robot velocity`. Aiming straight at the target therefore misses, by
more the faster you go.

The fix is to aim at a **virtual target**, displaced by `-robotVelocity × flightTime`. Shoot at that
as if standing still, the robot's velocity gets added back in by physics, and the ball arrives at the
real target.

### The derivation

With `p0` = muzzle, `pt` = target, `vr` = robot field velocity, `Vm` = muzzle velocity, `G` = gravity:

```
pt = p0 + (Vm + vr)*t + (0,0,-G)*t²/2
```

Rearranged, with virtual target `pv = pt - vr*t`:

```
horizontal:   Vm_h = (pv_h - p0_h)/t        so |Vm_h| = d/t, aimed at pv
vertical:     Vm_z = h/t + G*t/2            where h = pt_z - p0_z
```

A fixed hood angle `θ` ties those together:

```
tan θ = Vm_z / |Vm_h| = (h + G*t²/2) / d

   =>   t = sqrt( 2*(d*tan θ - h) / G )
```

**For a fixed hood, flight time falls straight out of the geometry** — no search needed. Then
`muzzleSpeed = (d/t)/cos θ`, and the azimuth is just the bearing to the virtual target.

The only circularity left is that `d` depends on `t` (through the virtual target) and `t` depends on
`d`. So iterate: start at `t = 0`, compute `d`, get `t`, repeat. It converges in two or three passes
because `vr*t` is small next to `d`.

That is all of [`Ballistics.solve()`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Ballistics.java).

### When there is no solution

`d*tanθ - h ≤ 0` means the target sits at or above the launch line — unreachable at that hood angle,
no matter the speed. The solver reports this rather than returning a garbage number, along with
"needs more RPM than the shooter has" and "turret cannot reach that angle". Each one tells the driver
something actionable: *get closer*, *get closer*, *rotate the chassis*.

### What is deliberately not modelled

**Air drag.** Real, and non-trivial on a 25 g ball. But a proper drag model needs a coefficient you
cannot measure without a wind tunnel, so instead there is one empirical term per ball type,
`BallType.dragPerInch`, applied as a speed scale that grows with range. Crude, but it is one number
you can tune in an afternoon — and drag is the only effect that should produce a *range-dependent*
error, which makes it identifiable.

**Tune it in this order**, or you will use one wrong number to hide another:

1. `transferEfficiency` first, from the flat-shot test in `T5`. It has no range dependence.
2. `dragPerInch` second, only once efficiency is right and long shots are still dropping low.

---

## Part 2: what odometry contributes beyond position

Three distinct jobs, and it is worth knowing them separately because each fails differently.

### 1. The velocity for the lead

`getFieldVelX()` / `getFieldVelY()`. Directly the `vr` above.

**One thing you must verify:** goBILDA documents the velocity getters as *"X (forward) velocity"* —
which sounds robot-relative — but documents *position* the same way, and position is unambiguously a
field coordinate. The labels come from the pod names and prove nothing. So
[`T3 Velocity Frame Test`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tuning/T3_VelocityFrameTest.java)
measures it: point the robot along field +Y and push it forward; whichever axis carries the signal
tells you the frame.

Get it wrong and nothing crashes. The turret just leads correctly along one axis and **backwards**
along the other. Twenty seconds now, or an unexplained miss pattern all season.

### 2. Bridging vision dropouts

Every accepted sighting is converted into a **field coordinate** and stored. When vision drops out —
the turret slews, an opposing robot crosses the view, the tag leaves frame — that stored field
coordinate is converted *back* into a fresh robot-relative aim using current odometry.

The turret keeps tracking through gaps of a second or more. Without this, every occlusion is a lost
shot. Verify it in `T4`: cover the camera, watch "live" go false while the solution keeps updating.

### 3. Latency correction

A webcam frame is 30–100 ms old by the time you process it. At 60 in/s that is up to 6 inches of
stale position.

`Odometry` keeps half a second of pose history, and `HiveTracker` pairs each detection with
`detection.frameAcquisitionNanoTime` — the pose from when the **shutter opened**, not now. Without it,
a sighting taken mid-turn places the target somewhere the target never was.

### 4. (Bonus) cancelling chassis rotation

A plain position PID always lags when the chassis spins, because the target angle in the robot frame
moves as fast as the chassis turns — and the PID only reacts *after* the error exists.

`Turret` adds `-headingVelocity × KV` as feedforward, so the correction happens before the error does.
The Pinpoint's fused IMU gives that rotation rate at **1500 Hz**, far cleaner than differentiating
heading in a 50 Hz loop. A concrete thing odometry does for the turret that vision cannot.

---

## Part 3: the colour sensor and shooter power

`Ballistics.solve()` produces the **ball exit speed** physics requires. That number does not care what
kind of ball it is. What differs is the **RPM needed to achieve it**:

```
rpm = 60 * muzzleSpeed / (transferEfficiency * π * flywheelDiameter)
```

NECTAR is 1.65× the mass and 1.29× the diameter of POLLEN, so more energy is lost to momentum transfer
and to compressing the ball. Its `transferEfficiency` comes out lower, so it needs **more RPM** —
which is the behaviour you wanted, arrived at from the physics rather than bolted on.

Why express it this way rather than "add N rpm for nectar": a fixed RPM bump is only correct at the
distance where you measured it. An efficiency is correct at every distance, and it is one number per
ball type rather than a whole second distance-to-RPM table.

### The second job, which is easy to forget

Only **your own alliance's** NECTAR scores for you. POLLEN is fair game for anyone. Firing the
opponent's blue NECTAR into your red HIVE spends a cycle for nothing.

`BallType.isScorableBy(alliance)` and `BallSorter.shouldReject()` handle it, and `TeleOpMain`
auto-ejects. Set `RobotConstants.alliance` before every match — easiest is to duplicate `TeleOpMain`
into a red and a blue version so nobody has to remember.

---

## Part 4: the four-gate fire decision

```java
public boolean isReadyToFire() {
    return aimingEnabled
        && solution.valid          // in range, turret can reach
        && turret.isOnTarget()     // slew finished
        && shooter.isAtSpeed()     // and SETTLED, not just passing through
        && hiveTracker.hasTarget();// estimate fresh enough to believe
}
```

Four conditions, not one, because on a robot that is also driving it is very easy for one to be false
without the driver noticing. The trigger is a *request*; the code decides.

`isAtSpeed()` deserves a note: it requires the flywheel to have been within tolerance **continuously**
for ~0.12 s. A spinning-up wheel passes through its target on the way to overshooting, and a ball fed
at that instant gets a speed that is right for one millisecond and wrong for the rest of the contact.
Requiring it to settle costs a tenth of a second and buys real consistency.

Given the scoring — **HIVE TIP is 20 points against 2 per element in a CELL** — consistent repeated
shots into one CELL is what earns points. Every ball fired before the wheel settled is a ball that
does not count toward a tip.

---

## Suggested order of work

1. **Odometry, properly.** `T0`–`T3`, then `TeleOp Drive Only`. Do not move on until driving a hard
   lap of the field returns a pose within an inch or two.
2. **Vision, on a stationary robot.** `T4`. Confirm the target's field estimate holds still while you
   push the robot around. This is the test that catches wrong camera mount numbers.
3. **Shooter, stationary.** `T5` for efficiency, then aim real arcs and tune `dragPerInch`. Get
   stationary shots landing reliably.
4. **Colour sensor.** `T6`. Confirm POLLEN and NECTAR get different RPMs, and that the opponent's
   NECTAR is rejected.
5. **Turn on the lead.** Only now. Drive slowly past the HIVE while shooting and watch where the
   misses go. If they trail behind the target, the lead is too small; if ahead, too large. Check
   `T3`'s answer before touching any gain.

Steps 1 and 2 are where the season is won. A turret with perfect ballistics on top of odometry that
drifts two inches per lap will miss, and you will blame the ballistics.
