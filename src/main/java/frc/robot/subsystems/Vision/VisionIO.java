// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /** Creates a new VisionIO. */
  @AutoLog
  public static class VisionIOInputs {
    public double tx;
    public double txAdjusted;
    public double ty;
    public double tyAdjusted;
    public double tv;
    public double pipeline;
    public double tagID;
    public Pose2d estimatedPose;
    public double timestampSeconds;
    public int[] targetIds;
    public double[] distancesToTargets;
    public boolean poseUpdated;
    public Pose3d[] tagPoses;
  }

  public void updateInputs(VisionIOInputs inputs);

  public void setLEDMode(int mode);

  public void setPipeline(int pipeline);

  public void takeSnapshot();

  public void resetSnapshot();
}
