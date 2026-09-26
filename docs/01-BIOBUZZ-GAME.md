# BIOBUZZ 2026-27 — what the software needs to know

Written for: your team's programmers and strategy lead.

BIOBUZZ presented by RTX is the 2026-27 FIRST Tech Challenge game, part of the **FIRST CANOPY**
season. Kickoff was **12 September 2026**.

> **Verify the numbers below against the manual before you build strategy on them.** They are
> assembled from the official manual, AndyMark's part listings and FIRST's own materials, but the
> manual is the only authority and it gets revised by Team Updates through the season. Direct links
> are at the bottom. There is also an official
> [Competition Manual chatbot](https://ftc-cmchatbot.firstinspires.org/) that answers rule questions
> with citations.

---

## The scoring elements — and why your shooter must treat them differently

| | **POLLEN** | **NECTAR** |
|---|---|---|
| Diameter | **2.8 in** | **3.6 in** |
| Weight | **0.055 lb** (~25 g) | **0.091 lb** (~41 g) |
| Colour | yellow | red or blue |
| Count per field | 40 | 8 red + 8 blue |
| Who can score it | **either alliance** | **only the matching alliance** |
| AndyMark part | `am-5851_yellow` | `am-5852_red` / `am-5852_blue` |

POLLEN is physically similar to last season's DECODE Artifacts, so DECODE-era shooter designs are a
reasonable starting point for it.

**Your instinct was right: NECTAR is bigger and heavier.** Specifically it is **1.65× the mass** and
**1.29× the diameter**. Two separate effects push the same way:

1. **Mass.** A heavier ball takes more momentum out of the flywheel during the brief contact, so it
   leaves slower at the same RPM.
2. **Compression.** A 3.6 in ball squeezed through a gap sized for a 2.8 in ball is compressed much
   harder. That improves grip (less slip) but costs more energy deforming the ball.

Net effect on nearly every FTC shooter: **NECTAR needs more RPM than POLLEN for the same distance.**

How the code handles it — see [`BallType.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/BallType.java):
rather than "add N rpm for nectar", each ball type carries a **transfer efficiency** (what fraction of
the flywheel's surface speed the ball actually leaves with). The ballistics solver computes the ball
speed physics demands; the efficiency converts that into an RPM. One tuned number per ball type, and
it stays correct at every distance instead of only where you happened to measure.

### The half of the colour sensor that is easy to forget

Only your own alliance's NECTAR scores for you. POLLEN is fair game for anyone. So the colour sensor
has **two** jobs:

1. Pick the shooter RPM.
2. **Reject the opponent's NECTAR** instead of firing it into your own HIVE and spending a cycle for
   nothing.

`BallType.isScorableBy(alliance)` and `BallSorter.shouldReject()` cover the second, and
`TeleOpMain` auto-ejects on it.

---

## Field elements

### The HIVE and its CELLS

- The HIVE is **bi-stable**: it holds its position until enough POLLEN or NECTAR has been LAUNCHED
  into the upward-facing CELL, at which point it **TIPS**.
- **On the bottom face of each CELL is a unique AprilTag Cluster of 4 distinct AprilTags.**
- A CELL returns **one cluster detection** when at least one member tag is visible. The member tags
  do **not** also appear as individual detections.
- **The cluster origin sits at the centre of the CELL opening** — FIRST placed it there deliberately
  so you can aim straight at it.

### FLOWERS, GARDEN

- Alliances score POLLEN and NECTAR into their HIVES and FLOWERS.
- **FLOWER scoring does not open until the last minute of the match.**

---

## Match structure

| Period | Length |
|---|---|
| AUTO | 30 s |
| Transition | 8 s |
| TELEOP | 2 min |
| (endgame — FLOWERS open, remaining NECTAR released) | last 60 s of TELEOP |

---

## Scoring

**Treat this table as provisional and confirm against manual section 10.**

| Action | AUTO | TELEOP |
|---|---|---|
| LEAVE (per robot) | 3 | — |
| PARK (per robot) | 5 | 5 |
| HIVE TIP | 20 | 20 |
| Element in CELL | — | 2 |
| Element in an owned FLOWER | — | 2 |
| Bottom NECTAR bonus | — | 5 |
| GARDEN | — | 1 |

### What this implies for software priorities

A **HIVE TIP is worth 20** against **2 per element in a CELL** — a factor of ten. Tipping requires
getting *enough* elements into the upward-facing CELL, so the thing that actually earns points is
**consistent repeated shots into one CELL**, not occasional spectacular ones.

That is a strong argument for the design already in the code: gate every shot on
`AimController.isReadyToFire()` (valid solution **and** turret settled **and** flywheel settled
**and** target estimate fresh) rather than letting the driver fire on a timer. A shot taken while the
flywheel is still spinning up is a ball that does not count toward a tip.

It is also an argument for **not** over-investing in shooting while moving at high speed early on.
Get stationary shots landing reliably first; the moving-shot solver is already written and waiting in
[`Ballistics.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Ballistics.java),
and turning it on is a matter of trusting your velocity numbers, not writing new code.

---

## Official links

| What | Where |
|---|---|
| Game & season hub | <https://www.firstinspires.org/programs/ftc/game-and-season> |
| All game materials | <https://ftc-resources.firstinspires.org/ftc/game> |
| Competition Manual (PDF) | <https://ftc-resources.firstinspires.org/ftc/game/manual> |
| Competition Manual (HTML) | <https://ftc-resources.firstinspires.org/ftc/game/cm-html> |
| **Manual §8 Game Overview** | <https://ftc-resources.firstinspires.org/ftc/game/manual-08> |
| **Manual §9 ARENA** (field, AprilTags) | <https://ftc-resources.firstinspires.org/ftc/game/manual-09> |
| **Manual §10 Game Details** (scoring) | <https://ftc-resources.firstinspires.org/ftc/game/manual-10> |
| Manual §12 Robot Construction Rules | <https://ftc-resources.firstinspires.org/ftc/game/manual-12> |
| Manual chatbot (cites rules) | <https://ftc-cmchatbot.firstinspires.org/> |
| Team Updates — **read these weekly** | <https://ftc-resources.firstinspires.org/ftc/game/tu-combined> |
| Team Q&A | <https://ftc-resources.firstinspires.org/ftc/game/q-a-system> |
| Game animation | <https://youtu.be/sUH3z5a5S9I> |
| Field walkthrough | <https://www.youtube.com/watch?v=47X9sYnPijw> |
| Field setup guide | <https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide> |
| Scoring elements (specs, order) | <https://andymark.com/products/biobuzz-scoring-elements> |
| Full game sets | <https://andymark.com/collections/first-tech-challenge-2026-2027> |
| Skill Builders (7 mini-games) | <https://community.firstinspires.org/introducing-first-tech-challenge-skill-builders> |

**Team Updates matter to you specifically.** Manual revisions have already reached TU02, and the
version string on the manual pages reads `TU02`. Scoring values and field details can change; if a
tag ID or a scoring rule shifts, your code changes with it.
