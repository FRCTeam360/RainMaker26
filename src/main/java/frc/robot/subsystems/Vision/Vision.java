// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  /** Maximum acceptable IMU roll or pitch deviation from camera mount (degrees). */
  private static final double IMU_ORIENTATION_THRESHOLD_DEG = 45.0;

  /** Maximum acceptable tag observed roll or pitch deviation from camera mount (degrees). */
  private static final double TAG_ORIENTATION_THRESHOLD_DEG = 45.0;

  private static final String STICKY_STOP_DASHBOARD_PREFIX = "StickyStop/";

  private final Map<String, VisionIO> ios;
  private int totalDetections = 0;
  private int rejectedMeasurements = 0;
  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private Timer snapshotTimer = new Timer();
  private List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

  /** Per-Limelight sticky stop flags. Once set, permanently rejects poses until manually reset. */
  private final Map<String, Boolean> stickyStopFlags = new HashMap<>();

  private final String VISION_LOGGING_PREFIX = "Vision/";

  private final Map<String, String> cachedLogKeys;

  private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP =
      new InterpolatingMatrixTreeMap<>();

  static {
    // Very low standard deviations = high confidence = vision dominates pose estimation
    // X and Y in meters, rotation in radians
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(
        0.1,
        VecBuilder.fill(
            0.25, 0.25, 999999.0)); // Close tags ( at 10 cm): very high confidence (50 cm std dev)
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(
        8.0,
        VecBuilder.fill(
            3.0, 3.0, 999999.0)); // Far tags (8 meters):  very low confidence (5 meter cm std dev)
  }

  /** Creates a new Vision. */
  public Vision(Map<String, VisionIO> visionIos) {
    this.ios = visionIos;
    // Creates the same number of inputs as vision IO layers
    visionInputs = new HashMap<>();
    for (String key : visionIos.keySet()) {
      visionInputs.put(key, new VisionIOInputsAutoLogged());
    }
    cachedLogKeys = new HashMap<>();
    for (String key : visionIos.keySet()) {
      cachedLogKeys.put(key, VISION_LOGGING_PREFIX + key);
      stickyStopFlags.put(key, false);
    }
    enableIMUSeeding();
  }

  public void turnOnLights(String name) {
    Optional.ofNullable(ios.get(name)).ifPresent(io -> io.setLEDMode(3));
  }

  public void turnOffLights(String name) {
    Optional.ofNullable(ios.get(name)).ifPresent(io -> io.setLEDMode(1));
  }

  public void blinkLights(String name) {
    Optional.ofNullable(ios.get(name)).ifPresent(io -> io.setLEDMode(2));
  }

  public boolean isTargetInView(String name) {
    return Optional.ofNullable(visionInputs.get(name)).map(input -> input.tv == 1.0).orElse(false);
  }

  public int getPipeline(String name) {
    return Optional.ofNullable(visionInputs.get(name)).map(input -> input.pipeline).orElse(0);
  }

  public void setPipeline(String name, int pipeline) {
    Optional.ofNullable(ios.get(name))
        .ifPresent(
            io -> {
              io.setPipeline(pipeline);
            });
  }

  public void takeSnapshot(String name) {
    Optional.ofNullable(ios.get(name))
        .ifPresent(
            io -> {
              io.takeSnapshot();
              Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
              snapshotTimer.stop();
              snapshotTimer.reset();
              snapshotTimer.start();
            });
  }

  public void resetSnapshot(String name) {
    Optional.ofNullable(ios.get(name))
        .ifPresent(
            io -> {
              io.resetSnapshot();
              Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
              snapshotTimer.stop();
            });
  }

  @Override
  public void periodic() {
    // Clear previous measurements to prevent unbounded growth
    acceptedMeasurements.clear();

    for (Map.Entry<String, VisionIO> entry : ios.entrySet()) {
      VisionIO io = entry.getValue();
      String key = entry.getKey();

      VisionIOInputsAutoLogged input = visionInputs.get(key);

      io.updateInputs(input);

      // Fill remaining array slots with first target to avoid 0,0 visualization lines
      if (input.targetCount > 0) {
        for (int i = input.targetCount; i < VisionIO.MAX_TAGS; i++) {
          input.targetIds[i] = input.targetIds[0];
          input.distancesToTargets[i] = input.distancesToTargets[0];
          input.tagPoses[i] = input.tagPoses[0];
        }
      }
      Logger.processInputs(cachedLogKeys.get(key), input);
    }

    for (Map.Entry<String, VisionIOInputsAutoLogged> entry : visionInputs.entrySet()) {
      String key = entry.getKey();
      VisionIOInputsAutoLogged input = entry.getValue();

      // Check if driver requested stickystop reset via dashboard toggle (replay-safe via inputs)
      if (Boolean.TRUE.equals(stickyStopFlags.get(key)) && !input.stickyStopDashboardActive) {
        resetStickyStop(key);
      }

      // Compute orientation threshold checks from raw inputs (derived values — logged as outputs)
      double cameraRollDeg = Math.toDegrees(input.cameraPoseRobotSpace.getRotation().getX());
      double cameraPitchDeg = Math.toDegrees(input.cameraPoseRobotSpace.getRotation().getY());

      boolean imuExceedsThreshold = false;
      if (input.hasIMU) {
        imuExceedsThreshold =
            Math.abs(input.imuRollDeg - cameraRollDeg) > IMU_ORIENTATION_THRESHOLD_DEG
                || Math.abs(input.imuPitchDeg - cameraPitchDeg) > IMU_ORIENTATION_THRESHOLD_DEG;
      }

      boolean tagExceedsThreshold =
          Math.abs(input.nearestTagObservedRollDeg - cameraRollDeg) > TAG_ORIENTATION_THRESHOLD_DEG
              || Math.abs(input.nearestTagObservedPitchDeg - cameraPitchDeg)
                  > TAG_ORIENTATION_THRESHOLD_DEG;

      Logger.recordOutput(
          VISION_LOGGING_PREFIX + key + "/IMUOrientationExceedsThreshold", imuExceedsThreshold);
      Logger.recordOutput(
          VISION_LOGGING_PREFIX + key + "/TagOrientationExceedsThreshold", tagExceedsThreshold);

      // Count total detections (pose updates attempted)
      if (input.poseUpdated) {
        totalDetections++;
      }

      // If this Limelight has been sticky-stopped, reject all measurements
      if (Boolean.TRUE.equals(stickyStopFlags.get(key))) {
        rejectedMeasurements++;
        Logger.recordOutput(VISION_LOGGING_PREFIX + key + "/StickyStop", true);
        continue;
      }

      // skip input if not updated
      if (!input.poseUpdated) {
        rejectedMeasurements++;
        continue;
      }

      // Trigger sticky stop when both thresholds are exceeded while enabled
      if (imuExceedsThreshold && tagExceedsThreshold && DriverStation.isEnabled()) {
        stickyStopFlags.put(key, true);
        rejectedMeasurements++;
        Logger.recordOutput(VISION_LOGGING_PREFIX + key + "/StickyStop", true);
        continue;
      }

      // Reject measurements when either threshold is exceeded
      if (imuExceedsThreshold || tagExceedsThreshold) {
        rejectedMeasurements++;
        continue;
      }

      Pose2d pose = input.estimatedPose;
      double timestamp = input.timestampSeconds;

      // Skip measurements that are not with in the field boundary
      if (isPoseOutOfBounds(pose)) {
        rejectedMeasurements++;
        continue;
      }

      // Get standard deviation based on distance to nearest tag (zero allocation)
      double closestTagDistance = Double.MAX_VALUE;
      for (int i = 0; i < input.targetCount; i++) {
        if (input.distancesToTargets[i] < closestTagDistance) {
          closestTagDistance = input.distancesToTargets[i];
        }
      }

      Matrix<N3, N1> cprStdDevs = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance);

      acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, cprStdDevs));
    }

    // Log rejection statistics
    Logger.recordOutput(VISION_LOGGING_PREFIX + "Total Detections", totalDetections);
    Logger.recordOutput(VISION_LOGGING_PREFIX + "Rejected Measurements", rejectedMeasurements);
    Logger.recordOutput(
        VISION_LOGGING_PREFIX + "Rejection Rate",
        totalDetections > 0 ? (double) rejectedMeasurements / totalDetections : 0.0);

    // Publish stickystop status to SmartDashboard for driver display (output only)
    for (String key : stickyStopFlags.keySet()) {
      SmartDashboard.putBoolean(
          STICKY_STOP_DASHBOARD_PREFIX + key, Boolean.TRUE.equals(stickyStopFlags.get(key)));
    }
  }

  /** Resets the sticky stop flag for a specific Limelight, re-enabling pose acceptance. */
  public void resetStickyStop(String name) {
    stickyStopFlags.put(name, false);
    Logger.recordOutput(VISION_LOGGING_PREFIX + name + "/StickyStop", false);
  }

  /** Resets all sticky stop flags, re-enabling pose acceptance for all Limelights. */
  public void resetAllStickyStops() {
    for (String key : stickyStopFlags.keySet()) {
      resetStickyStop(key);
    }
  }

  /** Returns whether the given Limelight has been sticky-stopped. */
  public boolean isStickyStopActive(String name) {
    return Boolean.TRUE.equals(stickyStopFlags.get(name));
  }

  /** Enables IMU seeding on all vision IO layers. Call during disabled. */
  public void enableIMUSeeding() {
    for (VisionIO io : ios.values()) {
      io.enableIMUSeeding();
    }
  }

  /** Enables IMU assist on all vision IO layers. Call when robot is enabled. */
  public void enableIMUAssist() {
    for (VisionIO io : ios.values()) {
      io.enableIMUAssist();
    }
  }

  /**
   * Sets the processing throttle on all vision IO layers.
   *
   * @param throttle number of frames to skip between processed frames (0 = full speed)
   */
  public void setThrottle(int throttle) {
    for (VisionIO io : ios.values()) {
      io.setThrottle(throttle);
    }
  }

  /**
   * @return Command that consumes vision measurements
   */
  public Command consumeVisionMeasurements(
      Consumer<List<VisionMeasurement>> visionMeasurementConsumer) {
    return run(() -> visionMeasurementConsumer.accept(acceptedMeasurements));
  }

  public static boolean isPoseOutOfBounds(Pose2d pose) {
    return pose.getX() < 0.0
        || pose.getX() > FieldConstants.fieldLength
        || pose.getY() < 0.0
        || pose.getY() > FieldConstants.fieldWidth;
  }
}
