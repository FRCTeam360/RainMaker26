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
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final Map<String, VisionIO> ios;

  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private Timer snapshotTimer = new Timer();
  List<VisionMeasurement> acceptedMeasurements = Collections.emptyList();

  private final String VISION_LOGGING_PREFIX = "Vision: ";

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
    for (String key : ios.keySet()) {
      VisionIO io = ios.get(key);
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      io.updateInputs(input);
      Logger.processInputs("Limelight: " + key, input);
    }

    List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

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

      // get standard deviation based on distance to nearest tag
      OptionalDouble closestTagDistance = Arrays.stream(input.distancesToTargets).min();

      Matrix<N3, N1> cprStdDevs =
          MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance.orElse(Double.MAX_VALUE));

      acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, cprStdDevs));
    }
    this.acceptedMeasurements = acceptedMeasurements;
  }

  /**
   * @return Command that consumes vision measurements
   */
  public Command consumeVisionMeasurements(
      Consumer<List<VisionMeasurement>> visionMeasurementConsumer) {
    return run(() -> visionMeasurementConsumer.accept(acceptedMeasurements));
  }
}
