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

  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private Timer snapshotTimer = new Timer();

  /** Pre-allocated list to avoid GC pressure in periodic loop. */
  private final List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

  /** Pre-computed log keys to avoid string concatenation in periodic loop. */
  private final Map<String, String> limelightLogKeys = new HashMap<>();

  private static final String LOG_SNAPSHOT = "Vision: snapshot";

  private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP =
      new InterpolatingMatrixTreeMap<>();

  static {
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(
        0.5, VecBuilder.fill(1.0, 1.0, 999999.0)); // n1 and n2 are for x and y, n3
    // is for angle
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(5.0, VecBuilder.fill(10.0, 10.0, 999999.0));
  }

  /** Creates a new Vision. */
  public Vision(Map<String, VisionIO> visionIos) {
    this.ios = visionIos;
    // Creates the same number of inputs as vision IO layers
    visionInputs = new HashMap<>();
    for (String key : visionIos.keySet()) {
      visionInputs.put(key, new VisionIOInputsAutoLogged());
      // Pre-compute log keys to avoid string concatenation in periodic loop
      limelightLogKeys.put(key, "Limelight: " + key);
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
              Logger.recordOutput(LOG_SNAPSHOT, true);
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
              Logger.recordOutput(LOG_SNAPSHOT, false);
              snapshotTimer.stop();
            });
  }

  @Override
  public void periodic() {
    for (String key : ios.keySet()) {
      VisionIO io = ios.get(key);
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      io.updateInputs(input);
      Logger.processInputs(limelightLogKeys.get(key), input);
    }

    // Clear and reuse pre-allocated list to avoid GC pressure
    acceptedMeasurements.clear();

    for (String key : visionInputs.keySet()) {
      VisionIOInputsAutoLogged input = visionInputs.get(key);
      // skip input if not updated
      if (!input.poseUpdated) continue;

      Pose2d pose = input.estimatedPose;
      double timestamp = input.timestampSeconds;

      // Skip measurements that are not with in the field boundary
      if (pose.getX() < 0.0
          || pose.getX() > Constants.FIELD_LAYOUT.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > Constants.FIELD_LAYOUT.getFieldWidth()) continue;

      // Get minimum distance to targets without stream allocation
      double closestTagDistance = Double.MAX_VALUE;
      for (double dist : input.distancesToTargets) {
        if (dist < closestTagDistance) {
          closestTagDistance = dist;
        }
      }

      Matrix<N3, N1> cprStdDevs = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance);

      acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, cprStdDevs));
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
