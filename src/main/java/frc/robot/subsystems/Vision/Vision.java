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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final Map<String, VisionIO> ios;
  private int totalDetections = 0;
  private int rejectedMeasurements = 0;
  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private Timer snapshotTimer = new Timer();
  private List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

  private final String VISION_LOGGING_PREFIX = "Vision: ";

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

    for (String key : ios.keySet()) {
      VisionIO io = ios.get(key);
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

      Logger.processInputs("Limelight: " + key, input.clone());
    }

    for (String key : visionInputs.keySet()) {
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      // Count total detections (pose updates attempted)
      if (input.poseUpdated) {
        totalDetections++;
      }

      // skip input if not updated
      if (!input.poseUpdated) {
        rejectedMeasurements++;
        continue;
      }

      Pose2d pose = input.estimatedPose;
      double timestamp = input.timestampSeconds;

      // Skip measurements that are not with in the field boundary
      if (pose.getX() < 0.0
          || pose.getX() > Constants.FIELD_LAYOUT.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > Constants.FIELD_LAYOUT.getFieldWidth()) {
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
  }

  /** Seeds the IMU on all vision IO layers. Call during disabled. */
  public void seedIMU() {
    for (VisionIO io : ios.values()) {
      io.seedIMU();
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
}
