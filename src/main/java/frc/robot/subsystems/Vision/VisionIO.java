// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /** Maximum number of AprilTags that can be tracked simultaneously. */
  public static final int MAX_TAGS = 12;

  /**
   * Returns the SmartDashboard key for a camera's sticky stop toggle.
   *
   * @param cameraName the NetworkTables name of the camera
   * @return the dashboard key (e.g., "StickyStop/limelight-right")
   */
  static String stickyStopDashboardKey(String cameraName) {
    return "StickyStop/" + cameraName;
  }

  /** Creates a new VisionIO. */
  @AutoLog
  public static class VisionIOInputs {
    public double tx;
    public double ty;
    public double tv;
    public int pipeline;
    public double tagID;
    public Pose2d estimatedPose;
    public double timestampSeconds;
    public boolean poseUpdated;

    // Fixed-size arrays (preallocated for max possible tags) to avoid allocations at 50Hz
    public int targetCount = 0; // Number of valid entries in the arrays below
    public int[] targetIds = new int[MAX_TAGS];
    public double[] distancesToTargets = new double[MAX_TAGS];
    public Pose3d[] tagPoses = initTagPoses();

    // Limelight 4 internal IMU orientation (degrees)
    public double imuRollDeg = 0.0;
    public double imuPitchDeg = 0.0;
    public double imuYawDeg = 0.0;

    // Nearest detected tag's observed roll and pitch in robot space (degrees)
    public double nearestTagObservedRollDeg = 0.0;
    public double nearestTagObservedPitchDeg = 0.0;

    // Whether this camera has an internal IMU (e.g., Limelight 4)
    public boolean hasIMU = false;

    // Camera pose relative to the robot center (from Limelight configuration)
    public Pose3d cameraPoseRobotSpace = new Pose3d();

    // Current value of the StickyStop dashboard toggle (read from SmartDashboard for replay safety)
    public boolean stickyStopDashboardActive = false;

    private static Pose3d[] initTagPoses() {
      Pose3d[] poses = new Pose3d[MAX_TAGS];
      for (int i = 0; i < MAX_TAGS; i++) {
        poses[i] = new Pose3d();
      }
      return poses;
    }
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
}
