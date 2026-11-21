// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Generic localization source used by the {@link Vision} subsystem.
 *
 * <p>Each implementation reports zero or more pose observations per loop. Observations carry a
 * {@link PoseObservationType} so {@code Vision} can apply source-specific filtering and trust.
 */
public interface VisionIO {
  /** Creates a new VisionIO. */
  @AutoLog
  class VisionIOInputs {
    /** Whether the device is currently reachable. */
    public boolean connected = false;

    /** Pose observations produced by this source since the last update. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /** Tag IDs observed this loop, for logging and visualization. Empty for tagless sources. */
    public int[] tagIds = new int[0];

    // Generic target/debug telemetry (not used for pose estimation)
    public boolean hasTarget = false;
    public double txDeg = 0.0;
    public double tyDeg = 0.0;
    public int pipeline = 0;
    public int primaryTagId = -1;
    public Pose3d cameraPose = new Pose3d();

    // Optional device telemetry (QuestNav)
    public double batteryPercent = -1.0;
    public double deviceLatencyMs = 0.0;

    // Optional IMU telemetry (Limelight 4)
    public boolean hasIMU = false;
    public double imuYawDeg = 0.0;
    public double imuPitchDeg = 0.0;
    public double imuRollDeg = 0.0;
    public double nearestTagObservedPitchDeg = 0.0;
    public double nearestTagObservedRollDeg = 0.0;
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double closestTagDistanceMeters,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    QUESTNAV
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setLEDMode(int mode) {}

  default void setPipeline(int pipeline) {}

  default void takeSnapshot() {}

  default void resetSnapshot() {}

  /** Enables IMU seeding from external orientation data. Only relevant for Limelight 4. */
  default void enableIMUSeeding() {}

  /** Enables IMU assist mode. Only relevant for Limelight 4. */
  default void enableIMUAssist() {}

  /**
   * Sets the processing throttle. Only relevant for Limelight 4.
   *
   * @param throttle number of frames to skip between processed frames (0 = full speed)
   */
  default void setThrottle(int throttle) {}

  /**
   * Resets this source's field reference to a known robot pose. Sources that maintain their own
   * field frame (QuestNav) should override this. Tag-based cameras do not need to.
   *
   * @param robotPose known robot pose on the field
   */
  default void setPose(Pose2d robotPose) {}

  String getName();
}
