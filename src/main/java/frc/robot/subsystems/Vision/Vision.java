// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WoodBotConstants;

public class Vision extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private Timer snapshotTimer = new Timer();

  private final Map<String, VisionIO> ios;

  private final Map<String, VisionIOInputsAutoLogged> visionInputs;

  private final String VISION_LOGGING_PREFIX = "Vision: ";

  /** Creates a new Vision. */
  public Vision(Map<String, VisionIO> visionIOs) {
    this.ios = visionIOs;

    visionInputs = new HashMap<>();
    for (String key : visionIOs.keySet()) {
      visionInputs.put(key, new VisionIOInputsAutoLogged());
    }
  }

  public void turnOnLights(String name) {
    ios.get(name).setLEDMode(3);
  }

  public void turnOffLights(String name) {
    ios.get(name).setLEDMode(1);
  }

  public void blinkLights(String name) {
    ios.get(name).setLEDMode(2);
  }

  public int getAprilTagID(String name) {
    return ios.get(name).getAprilTagID();
  }

  public double getTXRaw(String name) {
    return ios.get(name).getTXRaw();
  }

  public double getTYRaw(String name) {
    return ios.get(name).getTYRaw();
  }

  public double getTVRaw(String name) {
    return ios.get(name).getTVRaw();
  }

  // public double getPipeline(String name) {
  //   return ios.get(name).getPipeline();
  // }

  // public double getPipeline(String name) {
  //   return ios.get(name).getPipeline();
  // }

  public void setPipeline(String name, int pipeline) {
    if (ios.get(name).getPipeline() != pipeline) {
      ios.get(name).setPipeline(pipeline);
    }
  }

  public void takeSnapshot(String name) {
    ios.get(name).takeSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
    snapshotTimer.stop();
    snapshotTimer.reset();
    snapshotTimer.start();
  }

  public void resetSnapshot(String name) {
    ios.get(name).resetSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
    snapshotTimer.stop();
  }

  public boolean isOnTargetTX(String name, double goal) {
    if (Math.abs((getTXRaw(name) - goal)) < 1.0) {
      return true;
    }
    return false;
  }

  public boolean isOnTargetTY(String name, double goal) {
    if (Math.abs(getTYRaw(name) - goal) < 1.0) {
      return true;
    }
    return false;
  }

  public boolean isTargetInView(String name) {
    return getTVRaw(name) == 1;
  }

  public Command waitUntilTargetTxTy(String name, double goalTX, double goalTY) {
    return Commands.waitUntil(
        () -> isTargetInView(name) && isOnTargetTX(name, goalTX) && isOnTargetTY(name, goalTY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      turnOffLights(WoodBotConstants.LIMELIGHT_NAME);
    }

    long periodicStartTime = HALUtil.getFPGATime();

    for (String key : ios.keySet()) {
      VisionIO io = ios.get(key);
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      io.updateInputs(input);
      Logger.processInputs("Limelight: " + key, input);
    }

    // List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

    for (String key : visionInputs.keySet()) {
      VisionIOInputsAutoLogged input = visionInputs.get(key);
      if (!input.poseUpdated)
        continue;
    }

    Pose2d pose = input.estimatedPose;
    double timestamp = input.timestampSeconds;

    // if(pose.getX() < 0.0
    // || pose.getX() > Constants.FIELD_LAYOUT.getFieldLength())

    // OptionalDouble closestTagDistance =
    // Arrays.stream(input.distanceToTargets).min();

    // Matrix<N3, N1> cprStdDevs =
    // MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance.orElse(Double.MAX_VALUE));

    // acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, cprStdDevs));
    // }
    // this.acceptedMeasurements = acceptedMeasurements;
    // long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
    // Logger.recordOutput(VISION_LOGGING_PREFIX + "periodic loop time",
    // (periodicLoopTime / 1000.0));
    // }

    // public Command consumeVisionMeasurements(
    // Consumer<List<VisionMeasurement>> visionMeasurementConsumer) {
    // return CommandLogger.logCommand(
    // run(() -> visionMeasurementConsumer.accept(acceptedMeasurements)),
    // "Consume Vision Measurements");
    // }

    // i dont know what most of the above code does so I'm going to comment it out
    // for now until the basics are done
  }
}