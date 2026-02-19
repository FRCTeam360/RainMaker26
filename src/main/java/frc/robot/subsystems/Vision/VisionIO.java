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

    private static Pose3d[] initTagPoses() {
      Pose3d[] poses = new Pose3d[MAX_TAGS];
      for (int i = 0; i < MAX_TAGS; i++) {
        poses[i] = new Pose3d();
      }
      return poses;
    }
  }

  public void updateInputs(VisionIOInputs inputs);

  public void setLEDMode(int mode);

  public void setPipeline(int pipeline);

  public void takeSnapshot();

  public void resetSnapshot();
}
