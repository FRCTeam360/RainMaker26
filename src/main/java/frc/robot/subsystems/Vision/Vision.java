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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionIO.PoseObservation;
import frc.robot.subsystems.Vision.VisionIO.PoseObservationType;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * Fuses all localization sources into a single stream of vision measurements.
 *
 * <p>Sources can be tag-based cameras (Limelight, PhotonVision) or tagless odometry sources
 * (QuestNav). Each source reports {@link PoseObservation}s and {@code Vision} applies
 * source-specific filtering and standard deviations before handing measurements to the drivetrain.
 */
public class Vision extends SubsystemBase {
  private static final String VISION_LOGGING_PREFIX = "Vision/";

  // Tag-based filtering
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double MAX_Z_ERROR_METERS = 0.75;

  // QuestNav measurement trust (lower = trust more). Starting values from the AdvantageKit swerve
  // reference: 2 cm linear, 5 degrees angular.
  private static final double QUEST_LINEAR_STD_DEV_METERS = 0.02;
  private static final double QUEST_ANGULAR_STD_DEV_RADIANS = Math.toRadians(5.0);

  private final Map<String, VisionIO> ios;
  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private final Map<String, String> cachedLogKeys;
  private final Map<String, Alert> disconnectedAlerts;

  private int totalDetections = 0;
  private int rejectedMeasurements = 0;

  private Timer snapshotTimer = new Timer();
  private final List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

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
  public Vision(VisionIO... visionIos) {
    this.ios = new LinkedHashMap<>();
    for (VisionIO io : visionIos) {
      this.ios.put(io.getName(), io);
    }

    visionInputs = new LinkedHashMap<>();
    cachedLogKeys = new LinkedHashMap<>();
    disconnectedAlerts = new LinkedHashMap<>();
    for (String key : ios.keySet()) {
      visionInputs.put(key, new VisionIOInputsAutoLogged());
      cachedLogKeys.put(key, VISION_LOGGING_PREFIX + key);
      disconnectedAlerts.put(key, new Alert(key + " disconnected!", AlertType.kError));
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
    return Optional.ofNullable(visionInputs.get(name)).map(input -> input.hasTarget).orElse(false);
  }

  public int getPipeline(String name) {
    return Optional.ofNullable(visionInputs.get(name)).map(input -> input.pipeline).orElse(0);
  }

  public void setPipeline(String name, int pipeline) {
    Optional.ofNullable(ios.get(name)).ifPresent(io -> io.setPipeline(pipeline));
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

  /**
   * Resets the field reference of every localization source that maintains its own frame
   * (QuestNav). Tag-based cameras ignore this.
   *
   * @param robotPose known robot pose on the field
   */
  public void setPose(Pose2d robotPose) {
    for (VisionIO io : ios.values()) {
      io.setPose(robotPose);
    }
    Logger.recordOutput(VISION_LOGGING_PREFIX + "PoseReset", robotPose);
  }

  @Override
  public void periodic() {
    // Clear previous measurements to prevent unbounded growth
    acceptedMeasurements.clear();

    // Update every source first so all inputs are fresh before filtering
    for (Map.Entry<String, VisionIO> entry : ios.entrySet()) {
      String key = entry.getKey();
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      entry.getValue().updateInputs(input);
      Logger.processInputs(cachedLogKeys.get(key), input);
    }

    for (Map.Entry<String, VisionIOInputsAutoLogged> entry : visionInputs.entrySet()) {
      String name = entry.getKey();
      VisionIOInputsAutoLogged input = entry.getValue();

      disconnectedAlerts.get(name).set(!input.connected);

      int accepted = 0;
      int rejected = 0;

      for (PoseObservation observation : input.poseObservations) {
        totalDetections++;

        if (shouldReject(observation, input.connected)) {
          rejectedMeasurements++;
          rejected++;
          continue;
        }

        Pose2d pose = observation.pose().toPose2d();

        // Skip measurements that are not within the field boundary
        if (isPoseOutOfBounds(pose)) {
          rejectedMeasurements++;
          rejected++;
          continue;
        }

        acceptedMeasurements.add(
            new VisionMeasurement(observation.timestamp(), pose, getStdDevs(observation)));
        accepted++;
      }

      Logger.recordOutput(cachedLogKeys.get(name) + "/AcceptedMeasurements", accepted);
      Logger.recordOutput(cachedLogKeys.get(name) + "/RejectedMeasurements", rejected);
    }

    // Log rejection statistics
    Logger.recordOutput(VISION_LOGGING_PREFIX + "Total Detections", totalDetections);
    Logger.recordOutput(VISION_LOGGING_PREFIX + "Rejected Measurements", rejectedMeasurements);
    Logger.recordOutput(
        VISION_LOGGING_PREFIX + "Accepted Measurements", acceptedMeasurements.size());
    Logger.recordOutput(
        VISION_LOGGING_PREFIX + "Rejection Rate",
        totalDetections > 0 ? (double) rejectedMeasurements / totalDetections : 0.0);
  }

  /** Returns true if the observation should be rejected before reaching the pose estimator. */
  private static boolean shouldReject(PoseObservation observation, boolean connected) {
    if (!connected) {
      return true;
    }

    // QuestNav has no tags or ambiguity; pose validity is handled by the source itself.
    if (observation.type() == PoseObservationType.QUESTNAV) {
      return false;
    }

    int tagCount = observation.tagCount();
    if (tagCount == 0) {
      return true;
    }

    // Reject high-ambiguity single-tag solves
    if (tagCount == 1 && observation.ambiguity() > MAX_AMBIGUITY) {
      return true;
    }

    // Z must be realistic for a tag-based solve
    return Math.abs(observation.pose().getZ()) > MAX_Z_ERROR_METERS;
  }

  /** Returns the standard deviations to use for this observation. */
  private static Matrix<N3, N1> getStdDevs(PoseObservation observation) {
    if (observation.type() == PoseObservationType.QUESTNAV) {
      return VecBuilder.fill(
          QUEST_LINEAR_STD_DEV_METERS, QUEST_LINEAR_STD_DEV_METERS, QUEST_ANGULAR_STD_DEV_RADIANS);
    }

    double closestTagDistance = observation.closestTagDistanceMeters();
    if (!Double.isFinite(closestTagDistance) || closestTagDistance < 0.0) {
      closestTagDistance = Double.MAX_VALUE;
    }

    return MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance);
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
