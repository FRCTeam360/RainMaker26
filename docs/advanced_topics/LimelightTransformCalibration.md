# Limelight Camera Transform Calibration Guide

This guide walks through empirically calibrating the robot-space camera transforms for
`limelight-right` (~5 in right of center) and `limelight-left` (~7 in left of center),
both of which have significant pitch (~30°) and yaw (~20°). The goal is to make each
camera's MegaTag2 pose estimate converge on the true robot center.

---

## What you are adjusting

In the Limelight web UI under **Robot Space**, you set 6 values per camera:

| Parameter | Symbol | What it controls |
|-----------|--------|-----------------|
| Forward offset (X) | `tx` | Camera forward/back from robot center, meters |
| Left offset (Y) | `ty` | Camera left/right from robot center, meters |
| Up offset (Z) | `tz` | Camera height from robot center, meters |
| Roll | `rx` | Rotation around forward axis, degrees |
| Pitch | `ry` | Tilt up/down, degrees |
| Yaw | `rz` | Rotation left/right, degrees |

Errors in translation (tx/ty/tz) are **constant** regardless of tag distance.
Errors in rotation (rx/ry/rz) **grow with distance**. Use this to tell them apart.

---

## Prerequisites

- Robot on a flat, hard surface (not carpet if possible — wheels slip differently)
- At least 2 AprilTags visible from your test positions with known field coordinates
- Laptop connected to robot network
- AdvantageScope open and connected, or NT Explorer showing:
  - `Vision: limelight-right/estimatedPose` (X, Y, Rotation)
  - `Vision: limelight-left/estimatedPose` (X, Y, Rotation)
- A tape measure
- Painter's tape to mark robot positions on the floor

---

## Step 1: Determine your error type before touching anything

This step tells you whether you are dealing with a translation error, rotation error, or
both. **Do not skip this.** Adjusting the wrong parameters will make things worse.

### Setup

Pick a single AprilTag. Mark three positions on the floor directly in front of it:
- **Position A**: 1.5 m from the tag
- **Position B**: 3.0 m from the tag
- **Position C**: 5.0 m from the tag

At each position, align the robot so it faces the tag squarely (0° relative yaw).
Use the tag's known field coordinates to calculate the exact expected robot pose at each
distance.

> **Tip:** The 2026 field tag positions are in `FieldConstants.FIELD_LAYOUT`. Cross-reference
> with the game manual to verify.

### Measurement

At each position, read the reported pose from NT Explorer and record:

| Position | Expected X | Expected Y | Reported X (R) | Reported Y (R) | Reported X (L) | Reported Y (L) |
|----------|-----------|-----------|----------------|----------------|----------------|----------------|
| A (1.5m) | | | | | | |
| B (3.0m) | | | | | | |
| C (5.0m) | | | | | | |

Calculate the error vector (Reported − Expected) for each position.

### Interpret results

**If the X/Y error is roughly the same magnitude at A, B, and C:**
→ Primarily a **translation error**. Fix tx/ty/tz first. Go to Step 2a.

**If the error grows approximately in proportion to distance (error at C ≈ 3× error at A):**
→ Primarily a **rotation error**. Fix yaw/pitch first. Go to Step 2b.

**If both:** Fix rotation first (Step 2b), then re-run this diagnostic, then fix translation (Step 2a).

---

## Step 2a: Fixing translation errors (tx, ty, tz)

Translation errors appear as a constant offset at all distances. The correction is simple:

- If reported X is consistently **too high** → decrease `tx` (move camera back in robot space)
- If reported Y is consistently **too far right** → decrease `ty` (move camera left in robot space)
- Height (tz) errors mainly show up when the camera angle is steep; measure by comparing
  estimates at very different tag heights

Apply the correction, re-run the three-position diagnostic in Step 1, and confirm the
error is now **constant and near zero** at all distances before proceeding.

---

## Step 2b: Fixing rotation errors (rx, ry, rz)

Rotation errors are distance-dependent. Work on **one axis at a time** and re-run the
Step 1 diagnostic after each change.

### Yaw (rz) — Most common with a 20° yaw mount

**Signature:** The Y error (lateral) grows with distance. The X error may also grow but
less dramatically. The error direction rotates when you stand at different angles to the
same tag.

**Test:** At Position C (5m), rotate the robot ±10° and observe whether the reported pose
tracks correctly. If it drifts sideways as you rotate, yaw is off.

**Fix:** If the robot appears shifted **right** of where it is:
- Increase `rz` (rotate camera left in robot space) by 1° increments
- After each change, re-check all three positions
- Stop when the error at Position C is the same as at Position A

### Pitch (ry) — Most impactful with a 30° pitch mount

**Signature:** The X error (forward/back) grows with distance. Errors are worse when
the tag is near the floor or ceiling of the camera's FOV.

**Fix:** If the robot appears shifted **forward** of where it is:
- Decrease `ry` by 1° increments
- Pitch errors are sensitive — 1° at 5m ≈ 8.7cm error. Make small changes.

### Roll (rx) — Usually minor

**Signature:** Diagonal errors that swap between X and Y as you move around the tag.
Only adjust if yaw and pitch corrections have not resolved the remaining error.

---

## Step 3: Cross-camera validation

Once each camera is individually calibrated, the best validation is agreement between them.

### Procedure

Drive the robot to 5 different locations on the field (varied X, Y, and heading).
At each position, record the pose estimate from both cameras:

| Position | LL-Right X | LL-Right Y | LL-Left X | LL-Left Y | Disagreement (cm) |
|----------|-----------|-----------|----------|----------|-------------------|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |

**Target:** Disagreement < 5 cm at most positions. Occasional spikes up to 10–15 cm
near field edges (few tags visible) are acceptable.

**If disagreement grows with distance from tags:** One camera still has a residual
rotation error. Re-run Step 2b for that camera.

**If disagreement is constant across all positions:** One camera has a residual
translation offset. Re-run Step 2a for that camera.

### Using AdvantageScope for this

Enable the diagnostic logging (if implemented) which records
`Vision: limelight-right/estimatedPose` and `Vision: limelight-left/estimatedPose`
side-by-side. Drive a slow lap of the field and look for systematic divergence patterns
in the pose traces.

---

## Step 4: Recording final values

Once calibrated, record the final Robot Space values here for reference:

### limelight-right

| Parameter | CAD Value | Calibrated Value | Notes |
|-----------|-----------|-----------------|-------|
| Forward (X, m) | | | |
| Left (Y, m) | | | |
| Up (Z, m) | | | |
| Roll (rx, deg) | | | |
| Pitch (ry, deg) | | | |
| Yaw (rz, deg) | | | |

### limelight-left

| Parameter | CAD Value | Calibrated Value | Notes |
|-----------|-----------|-----------------|-------|
| Forward (X, m) | | | |
| Left (Y, m) | | | |
| Up (Z, m) | | | |
| Roll (rx, deg) | | | |
| Pitch (ry, deg) | | | |
| Yaw (rz, deg) | | | |

---

## Quick reference: error signatures

| Symptom | Likely cause | Parameter to adjust |
|---------|-------------|---------------------|
| Pose offset is the same near and far | Translation error | tx, ty, or tz |
| Pose error grows with tag distance | Rotation error | ry (pitch) or rz (yaw) |
| Lateral error grows with distance | Yaw error | rz |
| Forward/back error grows with distance | Pitch error | ry |
| Two cameras disagree and disagreement grows with distance | Rotation error on one camera | rz or ry of the worse camera |
| Two cameras disagree by a fixed amount everywhere | Translation error on one camera | tx or ty of the worse camera |
| Pose is correct at 2m but wrong at 5m | Rotation error dominant | Fix rotation before translation |
| Pose is wrong at 2m but error doesn't scale | Translation error dominant | Fix translation first |

---

## Notes on the 30°/20° mount

With significant pitch and yaw, small angular errors have large positional effects at
competition distances (3–6 m to tags):

| Angular error | Error at 3m | Error at 5m |
|--------------|-------------|-------------|
| 1° | ~5.2 cm | ~8.7 cm |
| 2° | ~10.5 cm | ~17.5 cm |
| 3° | ~15.7 cm | ~26.2 cm |

Make rotation adjustments in **0.5–1° increments** and re-measure after each. The
distance sweep (Step 1) is your signal — if adjusting by 1° makes the distance-dependent
error go away, you found the right value.
