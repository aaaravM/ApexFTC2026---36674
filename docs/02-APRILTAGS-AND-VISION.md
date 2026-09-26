# AprilTags ("the QR codes") and vision

## First, a naming thing worth getting right

The black-and-white squares on the FTC field are **AprilTags**, not QR codes. It matters practically,
not just pedantically:

| | QR code | AprilTag |
|---|---|---|
| Designed to carry | a URL or text payload | a single small ID number |
| Payload capacity | hundreds of bytes | ~587 distinct IDs in FTC's family |
| What you get back | the decoded string | the ID **plus full 6-DOF pose** — range, bearing, elevation, yaw |
| Robust at distance/angle | poorly | very |

The pose is the entire point. You are not reading data off the tag; you are measuring **where the tag
is relative to your camera**, in inches and degrees. That is what lets you aim a turret.

FTC uses the **36h11** family. Tag IDs run 0–586.

---

## Downloads — printable tags

All from FIRST's official field resources page, <https://ftc-resources.firstinspires.org/ftc/field>:

| What | Link |
|---|---|
| **AprilTags Printable — US Letter** | <https://ftc-resources.firstinspires.org/ftc/field/apriltag-us> |
| **AprilTags Printable — A4** | <https://ftc-resources.firstinspires.org/ftc/field/apriltag-a4> |
| **AprilTag Production Art** (vector, for accurate reprints) | <https://ftc-resources.firstinspires.org/ftc/field/apriltag-art> |
| Field CAD on Onshape — **measure CELL heights from this** | [Onshape](https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73) |
| Field CAD (STEP, .zip) | <https://ftc-resources.firstinspires.org/ftc/field/field-cad-step> |
| Field tour video | <https://youtu.be/47X9sYnPijw> |

> **Printing matters more than you would think.** Download the PDF and print it from a standalone PDF
> reader with scaling set to **"Actual size" / 100%**, never "Fit to page". A tag scaled to 97% makes
> every distance measurement read 3% long, which at 100 inches is three inches of aiming error that
> looks exactly like a badly tuned shooter. Measure a printed tag with a ruler afterwards and confirm.
>
> Tag size is measured **across the outside edge of the inner black border**, not the whole sheet.

### From GitHub

| What | Link |
|---|---|
| **All 587 tags in family 36h11, print-ready** — referenced by FTC's own docs | <https://github.com/rgov/apriltag-pdfs/tree/main/tag36h11/us_letter/100mm> |
| AprilTag reference implementation (the upstream project) | <https://github.com/AprilRobotics/apriltag> |
| FTC SDK (what this repo is built on) | <https://github.com/FIRST-Tech-Challenge/FtcRobotController> |
| FTC docs source | <https://github.com/FIRST-Tech-Challenge/ftcdocs> |
| Generate a tag at any size, SVG or PDF | <https://chaitanyantr.github.io/apriltag.html> |
| FTC test images for developing without a field | <https://ftc-docs.firstinspires.org/apriltag/opmode_test_images/opmode-test-images.html> |

The **test images** are worth a look before your field arrives — point the camera at a monitor showing
them and your detection pipeline runs end to end.

---

## The BIOBUZZ tag IDs — get them from the SDK, not from a web search

The specific BIOBUZZ tag IDs and cluster names are not published anywhere convenient. They live
compiled inside the SDK's tag library.

**Run [`T0 Tag Library Dump`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tuning/T0_TagLibraryDump.java).**
It needs **no hardware at all** — no motors, no camera, no field — and prints every tag ID, tag size
and cluster name straight out of `AprilTagGameDatabase.getBioBuzzTagLibrary()`. Definitive, and you
can run it today.

Write down the cluster `shortName` values. You need one of them for
`RobotConstants.Hive.TARGET_CLUSTER_SHORT_NAME`, so the turret locks onto your alliance's HIVE and not
the opponent's. Until you set it, the tracker accepts **any** cluster it sees.

---

## What is new in SDK 12.0: AprilTag Clusters

This is the biggest API change of the season, and it **breaks any AprilTag code written for an
earlier SDK**.

A **cluster** is a co-planar group of two or more tags with known positions relative to a shared
origin. In BIOBUZZ, each CELL carries a cluster of **4 distinct tags** on its bottom face.

Three things follow, and all three are good news:

1. **The origin can sit outside the tags.** In BIOBUZZ it is placed at the **centre of the CELL
   opening** — FIRST put it there specifically so you can aim at it. The pose you get back *is* your
   aim point. No offset maths needed.
2. **Clusters survive partial occlusion.** Full 6-DOF pose is recoverable from a single visible member
   tag. More visible tags means a better estimate, and `percentClusterFound` tells you how many you
   got.
3. **A CELL returns one cluster detection, not four singletons.** Member tags do not appear
   individually. So any singleton you detect is not a CELL.

### The code change

```java
// OLD -- will not compile against SDK 12.0
for (AprilTagDetection detection : currentDetections) {
    int id = detection.id;          // no longer on the base class
}

// NEW
for (AprilTagDetection detection : currentDetections) {
    if (detection instanceof AprilTagSingleDetection) {
        AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
        // single.id, single.metadata.name
    } else {
        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
        // cluster.metadata.shortName, cluster.percentClusterFound
    }
}
```

`ftcPose` and `frameAcquisitionNanoTime` stay on the base class, so range/bearing/elevation and the
timestamp work for both kinds. Migration notes: <https://ftc-docs.firstinspires.org/apriltag-clusters>

### And the constraint that shapes your whole design

> *"Unfortunately, since BIOBUZZ AprilTags move, they are not suitable for absolute Field
> Localization."* — SDK 12.0 release notes

The HIVE tips. The tags move with it. **You cannot use them to work out where your robot is.** See
[`03-ARCHITECTURE.md`](03-ARCHITECTURE.md) — this is the one assumption in the original plan that has
to change, and the fix is straightforward.

---

## About training a CV model to track the balls

Two things you need to know before spending time on this.

### 1. TensorFlow Object Detection has been removed from the FTC SDK

Straight from the release notes (version 10.0):

> *"Java classes and Blocks for TensorFlow Object Detection have been removed."*

There is no `TfodProcessor` in SDK 12.0. FIRST's old training toolchain
([`fmltc`](https://github.com/FIRST-Tech-Challenge/fmltc)) and their ML sample repo
([`ftc-object-detection`](https://github.com/FIRST-Tech-Challenge/ftc-object-detection)) are both
**archived**. Any tutorial you find describing on-Hub TFOD is for an SDK three seasons old.

### 2. For coloured spheres specifically, a neural net is the worse tool

This is not "settle for less" — it is the better engineering choice here:

| | Colour blob + circularity | CNN on the Control Hub |
|---|---|---|
| Frame rate | 30+ fps at 320×240 | single digits |
| CPU cost | a few percent | most of it — competing with your turret loop |
| Debuggable | see exactly which pixels matched, live on the preview | opaque |
| Training data needed | none | hundreds of labelled images |
| Handles varied shapes/poses | no | yes — **but your targets are spheres in 3 known colours** |

A CNN earns its keep when targets vary in shape, pose and appearance. POLLEN and NECTAR are uniformly
coloured spheres in yellow, red and blue, on a field with controlled lighting. There is nothing for a
CNN to learn that a hue threshold does not already know.

There is also a neat bonus: knowing the *colour* tells you the *diameter* (2.8 in vs 3.6 in), and
knowing the diameter lets you compute **range** from the blob's pixel radius by pinhole projection.
A CNN bounding box gives you the same thing only if you already know which class it found — so you
end up depending on the classification either way.

**So:** [`BallTracker.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallTracker.java)
uses the SDK's `ColorBlobLocatorProcessor` with its `BY_CIRCULARITY` filter, and returns type,
range and bearing per ball.

### If you still want a trained detector — do it on a Limelight 3A

This is the supported route and it is a genuinely good one:

- FTC-legal, and the SDK has a `Limelight3A` class plus a `SensorLimelight3A` sample
- It has **its own processor**, so it costs the Control Hub nothing
- You train on [Roboflow](https://roboflow.com) and flash the pipeline
- SDK 12.0 can **upload pipelines from robot code**, so you can version-control them in this repo

That gives your team the real ML experience without the Control Hub paying for it. What to avoid is
on-Hub inference — that is the option that is both hardest and slowest.

---

## Camera advice that will save you a match

**Lock the exposure short and raise the gain.** On auto-exposure, a camera in typical gym lighting
lengthens its exposure, and a moving robot then smears the tag into something undecodable — precisely
when you need it. `HiveTracker.setManualExposure()` does this; the constants are
`Camera.EXPOSURE_MS` and `Camera.GAIN`.

**Decimation trades range for frame rate.** From the SDK's own measurements with a C920:

| Decimation | 2 in tag | 5 in tag | Frame rate |
|---|---|---|---|
| 1 | 10 ft | — | 10 fps |
| 2 | 6 ft | — | 22 fps |
| 3 | 4 ft | 10 ft | 30 fps |

`HiveTracker` uses 2 as a compromise. You can change it mid-match if you want range while lining up
and rate while shooting.

**Mount the camera on the chassis, not the turret, to begin with.** A turret-mounted camera keeps the
target centred, which is lovely for accuracy — but it loses the target every time the turret slews,
and it couples two control loops together while you are still debugging both. Start chassis-mounted
with a wide field of view; `Camera.MOUNTED_ON_TURRET` is there when you want to switch.
