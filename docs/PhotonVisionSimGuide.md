# PhotonVision Simulation - Architecture Guide

This guide explains how PhotonVision simulation works and how we integrate it with our AdvantageKit IO architecture. It follows the same patterns described in the AdvantageKit Subsystem Guide.

> **Purpose:** High-level notes on how our PhotonVision simulation works and how it fits into the AdvantageKit IO pattern. This is meant for understanding, not a step-by-step tutorial.

---

## Table of Contents

1. [Why Simulate Vision?](#1-why-simulate-vision)
2. [The PhotonVision Simulation Architecture](#2-the-photonvision-simulation-architecture)
3. [Understanding the Components](#3-understanding-the-components)
4. [Creating VisionIOPhotonSim: Step-by-Step](#4-creating-visioniophotonsim-step-by-step)
5. [The Pose Supplier Pattern](#5-the-pose-supplier-pattern)
6. [Configuration and Tuning](#6-configuration-and-tuning)
7. [Visualization and Debugging](#7-visualization-and-debugging)
8. [Common Patterns and Gotchas](#8-common-patterns-and-gotchas)

---

## The Big Picture

PhotonVision simulation lets us test AprilTag detection without a real camera or field. The sim generates fake camera frames based on where the robot is on a virtual field, complete with latency, noise, and detection limits—just like a real camera.

```
┌─────────────────────────────────────────────────────────────────┐
│                      VisionSystemSim                            │
│  (The simulated "world" - contains the field and all tags)      │
│                                                                 │
│   ┌─────────────┐         ┌─────────────┐                       │
│   │ AprilTag 1  │         │ AprilTag 2  │   ...more tags        │
│   └─────────────┘         └─────────────┘                       │
│                                                                 │
│          ▲                                                      │
│          │ "sees" (based on pose, FOV, range)                   │
│          │                                                      │
│   ┌──────┴──────┐                                               │
│   │PhotonCamera │◄──── SimCameraProperties (FOV, noise, FPS)    │
│   │    Sim      │                                               │
│   └─────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                    │
                    │ updates
                    ▼
            ┌───────────────┐
            │ PhotonCamera  │  ← Same object used in real code!
            │ (NetworkTables)│
            └───────────────┘
                    │
                    │ reads results
                    ▼
            ┌───────────────┐
            │VisionIOPhoton │  ← Our IO implementation
            │     Sim       │
            └───────────────┘
```

---

## 1. Why Simulate Vision?

Vision simulation provides several key benefits for development:

### Core Benefits

- **Test Without Hardware**: Validate AprilTag detection logic without a camera or field
- **Repeatable Testing**: Same robot position always produces the same detections
- **Rapid Iteration**: Tune pose estimation without deploying to the robot
- **Integration Testing**: Test how vision affects autonomous routines in simulation
- **Edge Case Testing**: Easily test "what if I can only see one tag?" scenarios

### The Key Insight

Traditional vision code is impossible to test without hardware:

```java
// ❌ Traditional approach - requires physical camera
public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("frontCam");

  public Pose2d getEstimatedPose() {
    var result = camera.getLatestResult();  // Needs real camera!
    // ... process result
  }
}
```

With the IO pattern and simulation:

```java
// ✅ IO pattern approach - works anywhere
public class Vision extends SubsystemBase {
  private final VisionIO io;  // Could be real OR simulated

  public Vision(VisionIO io) {
    this.io = io;  // Injected - we don't care which implementation
  }
}
```

This is the same separation of concerns from the AdvantageKit guide, applied to vision.

---

## 2. The PhotonVision Simulation Architecture

PhotonVision provides a simulation framework that generates fake camera data based on where the robot is on a virtual field. Here's how the pieces fit together:

```
┌─────────────────────────────────────────────────────────────────┐
│                      VisionSystemSim                            │
│  "The virtual field" - contains all AprilTags and cameras       │
│                                                                 │
│   ┌─────────────┐         ┌─────────────┐                       │
│   │ AprilTag 1  │         │ AprilTag 2  │   ... all field tags  │
│   └─────────────┘         └─────────────┘                       │
│                                                                 │
│          ▲                                                      │
│          │ "sees" (based on pose, FOV, range, occlusion)        │
│          │                                                      │
│   ┌──────┴──────┐                                               │
│   │PhotonCamera │◄──── SimCameraProperties (resolution, FOV,    │
│   │    Sim      │                           noise, latency)     │
│   └─────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                    │
                    │ automatically updates
                    ▼
            ┌───────────────┐
            │ PhotonCamera  │  ← Same object used in real code!
            │ (NetworkTables)│
            └───────────────┘
                    │
                    │ reads via getAllUnreadResults()
                    ▼
            ┌───────────────┐
            │VisionIOPhoton │  ← Our IO implementation
            │     Sim       │
            └───────────────┘
```

### How It Fits Our IO Pattern

Just like motor subsystems, Vision uses dependency injection:

```
Vision Subsystem
      │
      ├── VisionIO (interface)
      │       │
      │       ├── VisionIOLimelight    ← Real Limelight hardware
      │       ├── VisionIOPhotonReal   ← Real PhotonVision camera
      │       └── VisionIOPhotonSim    ← Simulated PhotonVision ✓
      │
      └── Uses whichever IO is injected via RobotContainer
```

The `Vision` subsystem calls `io.updateInputs(inputs)` every loop and doesn't know—or need to know—whether it's talking to a real camera or a simulation.

---

## 3. Understanding the Components

### 3.1 VisionSystemSim - "The Virtual Field"

This is the container for the entire simulated world. Think of it as WPILib's `Field2d` but for 3D vision simulation.

```java
// Create the simulation world
VisionSystemSim visionSim = new VisionSystemSim("main");

// Add all AprilTags from the field layout
visionSim.addAprilTags(kTagLayout);
```

**Key Responsibilities:**
- Holds all vision targets (AprilTags)
- Manages all simulated cameras
- Must be updated with robot pose every loop via `visionSim.update(robotPose)`
- Publishes a `Field2d` for visualization

### 3.2 SimCameraProperties - "Camera Specs"

Defines the physical characteristics of your simulated camera:

```java
SimCameraProperties cameraProp = new SimCameraProperties();

// Resolution and field of view
cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));

// Detection noise (how "wrong" corner/center measurements can be)
cameraProp.setCalibError(0.35, 0.10);  // avg error, std dev in pixels

// Timing characteristics
cameraProp.setFPS(15);                  // Frame rate
cameraProp.setAvgLatencyMs(50);         // Pipeline processing delay
cameraProp.setLatencyStdDevMs(15);      // Latency jitter
```

**Why This Matters:**
- Realistic FPS means you won't get vision updates every robot loop
- Realistic latency tests your latency compensation code
- Realistic noise tests your filtering and rejection logic

### 3.3 PhotonCameraSim - "The Simulated Camera"

The bridge between the simulation world and the `PhotonCamera` your code uses:

```java
// The PhotonCamera your real code would use
PhotonCamera camera = new PhotonCamera("frontCam");

// Wrap it with simulation - this updates the PhotonCamera automatically
PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

// Limit detection range (default is infinite!)
cameraSim.setMaxSightRange(5.0);  // meters

// Add to the simulation world with the robot-to-camera transform
visionSim.addCamera(cameraSim, kRobotToCam);
```

**The Magic:** When you call `visionSim.update(robotPose)`, the `PhotonCameraSim` calculates what tags are visible and updates the linked `PhotonCamera` with simulated results. Your vision code reads from `PhotonCamera` as normal.

### 3.4 Transform3d kRobotToCam - "Where's the Camera?"

The simulation needs to know where the camera is mounted:

```java
// Camera is 0.5m forward, 0.5m up from robot center, facing forward
public static final Transform3d kRobotToCam = new Transform3d(
    new Translation3d(0.5, 0.0, 0.5),  // x (forward), y (left), z (up)
    new Rotation3d(0, 0, 0)             // roll, pitch, yaw
);
```

This transform is used for:
- Determining what the camera can see from the robot's position
- Converting camera-relative poses to field-relative poses

---

## 4. Creating VisionIOPhotonSim: Step-by-Step

Let's walk through our implementation.

### Step 1: Define Constants

File: `subsystems/Vision/VisionSimConstants.java`

```java
public class VisionSimConstants {
    public static class VisionPhotonSim {
        public static final String kCameraName = "YOUR CAMERA NAME";

        // Camera mounted 0.5m forward, 0.5m up, facing forward
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // Load the current season's AprilTag layout
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
```

### Step 2: Create the IO Implementation

File: `subsystems/Vision/VisionIOPhotonSim.java`

```java
public class VisionIOPhotonSim implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final Supplier<Pose2d> robotPoseSupplier;

  // Simulation objects
  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    // Create camera and pose estimator (same as real implementation would)
    camera = new PhotonCamera(kCameraName);
    photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    // --- Simulation-specific setup ---

    // Create the virtual field
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(kTagLayout);

    // Configure camera properties
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    // Create and configure simulated camera
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    cameraSim.setMaxSightRange(5.0);  // 5 meter detection limit

    // Add camera to the simulation world
    visionSim.addCamera(cameraSim, kRobotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulation with current robot pose
    visionSim.update(robotPoseSupplier.get());

    // Read results from the (now-updated) PhotonCamera
    var results = camera.getAllUnreadResults();

    // Process results and fill inputs...
    // (See full implementation in VisionIOPhotonSim.java)
  }
}
```

### Step 3: Wire It Up in RobotContainer

```java
// In RobotContainer.java
switch (Constants.currentMode) {
  case REAL:
    vision = new Vision(Map.of(
        "limelight", new VisionIOLimelight()
    ));
    break;

  case SIM:
    vision = new Vision(Map.of(
        "photonSim", new VisionIOPhotonSim(() -> drivetrain.getState().Pose)
    ));
    break;

  case REPLAY:
    vision = new Vision(Map.of());  // Replay uses logged data
    break;
}
```

---

## 5. The Pose Supplier Pattern

### The Problem

Unlike motor simulations that are self-contained, vision simulation needs external information: **where is the robot on the field?**

### The Solution

We pass a `Supplier<Pose2d>` to the constructor:

```java
public VisionIOPhotonSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    // ...
}
```

This supplier is called every loop in `updateInputs()`:

```java
@Override
public void updateInputs(VisionIOInputs inputs) {
    // Get the current robot pose and update simulation
    Pose2d robotPose = robotPoseSupplier.get();
    visionSim.update(robotPose);
    // ...
}
```

### Critical: Use Simulated Pose, Not Estimated Pose

```java
// ✅ Correct - use the simulated/ground-truth pose
new VisionIOPhotonSim(() -> drivetrain.getState().Pose)

// ❌ Wrong - creates a feedback loop!
new VisionIOPhotonSim(() -> poseEstimator.getEstimatedPosition())
```

**Why?** If you use the estimated pose:
1. Sim generates detections based on estimated pose
2. Vision updates the pose estimator
3. Estimated pose changes
4. Sim generates different detections
5. Feedback loop causes oscillation or drift

The simulation needs **ground truth**—where the robot *actually is* in the simulated world.

---

## 6. Configuration and Tuning

### Detection Range

By default, `PhotonCameraSim` can see tags at any distance. This is unrealistic:

```java
// Limit to 5 meters (adjust based on your camera/tag size)
cameraSim.setMaxSightRange(5.0);
```

### Camera Properties

Tune these to match your real camera behavior:

| Property | Method | Typical Value | Effect |
|----------|--------|---------------|--------|
| Resolution | `setCalibration(w, h, fov)` | 960x720, 90° | Higher = slower but more accurate |
| Calibration Error | `setCalibError(avg, stdDev)` | 0.35, 0.10 | Detection noise in pixels |
| Frame Rate | `setFPS(fps)` | 15-30 | How often you get new data |
| Latency | `setAvgLatencyMs(ms)` | 30-50 | Pipeline processing delay |
| Latency Jitter | `setLatencyStdDevMs(ms)` | 10-15 | Latency variation |

### What Gets Simulated vs. What Doesn't

**✅ Simulated:**
- Camera field of view and resolution
- Target detection based on geometry (position, angle, distance)
- Detection noise (corner positions, yaw/pitch errors)
- Ambiguity for single-tag pose estimation
- Latency and frame timing
- Multi-tag pose estimation

**❌ NOT Simulated:**
- Image thresholding (exposure, gain, brightness settings)
- Pipeline switching (all pipelines behave identically)
- LED modes (no effect in simulation)
- Snapshots

---

## 7. Visualization and Debugging

### Field2d Widget

The `VisionSystemSim` publishes a `Field2d` to NetworkTables:

**Location:** `/VisionSystemSim-main/Sim Field`

This shows:
- Robot pose (as the simulation sees it)
- Camera frustum (field of view cone)
- Detected tag positions

### Camera Streams

Enable MJPEG streams for visual debugging:

```java
cameraSim.enableRawStream(true);        // Raw camera feed
cameraSim.enableProcessedStream(true);  // With detections drawn
cameraSim.enableDrawWireframe(true);    // 3D field overlay (expensive!)
```

**Stream Locations:**
- Raw: `http://localhost:1181`
- Processed: `http://localhost:1182`

**Note:** `enableDrawWireframe(true)` is resource-intensive. Use it for debugging, but disable it for normal testing.

### Adding Custom Visualization

In `updateInputs()`, we update a `Field2d` object with our pose estimate:

```java
if (visionEst.isPresent()) {
    getSimDebugField()
        .getObject("VisionEstimation")
        .setPose(est.estimatedPose.toPose2d());
}
```

This lets you compare the vision estimate against the ground-truth robot pose.

---

## 8. Common Patterns and Gotchas

### 8.1 "Camera sees nothing"

**Checklist:**
1. Are you calling `visionSim.update(pose)` in `updateInputs()`?
2. Is `setMaxSightRange()` set too low?
3. Is `kRobotToCam` pointing the camera away from tags?
4. Is the robot pose valid (not `null` or `NaN`)?

### 8.2 "Pose estimates are way off"

**Likely causes:**
- Using estimated pose instead of simulated pose for `visionSim.update()`
- `AprilTagFieldLayout` doesn't match the field you expect
- `kRobotToCam` transform is incorrect

### 8.3 "Tags detected at impossible distances"

The default configuration has no range limit:

```java
// Fix: Add a realistic range limit
cameraSim.setMaxSightRange(5.0);
```

### 8.4 "Latency seems wrong in logs"

The simulation adds latency to timestamps intentionally. Pose estimates will appear "old" just like real camera data. This is correct behavior—it tests your latency compensation code.

### 8.5 Multiple Cameras

You can add multiple cameras to one `VisionSystemSim`:

```java
visionSim.addCamera(frontCameraSim, kRobotToFrontCam);
visionSim.addCamera(backCameraSim, kRobotToBackCam);
```

All cameras share the same virtual field and are updated together.

### 8.6 Dynamic Camera Mounting (Turrets)

For cameras on moving mechanisms:

```java
// In periodic, update the camera transform
Rotation3d turretRotation = new Rotation3d(0, 0, turretAngle);
Transform3d currentTransform = new Transform3d(
    kBaseCamTranslation.rotateBy(turretRotation),
    kBaseCamRotation.rotateBy(turretRotation)
);
visionSim.adjustCamera(cameraSim, currentTransform);
```

---

## Summary

PhotonVision simulation provides:

1. **Hardware-Free Testing**: Validate vision code without cameras or a field
2. **IO Pattern Integration**: `VisionIOPhotonSim` implements the same interface as real implementations
3. **Realistic Behavior**: Configurable noise, latency, and detection limits
4. **Easy Debugging**: Built-in `Field2d` and camera stream visualization

The key pattern is the **pose supplier**—the simulation needs ground-truth robot position to calculate what the camera can see. Pass the simulated drivetrain pose, never the estimated pose.

---

## Quick Reference

| Component | Our Class/Constant |
|-----------|-------------------|
| IO Implementation | `VisionIOPhotonSim` |
| Camera Name | `VisionSimConstants.VisionPhotonSim.kCameraName` |
| Camera Mount | `VisionSimConstants.VisionPhotonSim.kRobotToCam` |
| Tag Layout | `VisionSimConstants.VisionPhotonSim.kTagLayout` |
| Max Sight Range | 5.0 meters (set in constructor) |

---

## Further Reading

- [PhotonVision Simulation Docs](https://docs.photonvision.org/en/latest/docs/simulation/index.html)
- [WPILib Simulation Overview](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- Our `AdvantageKit_Subsystem_Guide.md` for the IO pattern details
