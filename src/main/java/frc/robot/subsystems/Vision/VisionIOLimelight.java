// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final String name;
  private final DoubleSupplier gyroAngleSupplier;
  private final DoubleSupplier gyroAngleRateSupplier;
  private final boolean isLimelight4;

  private boolean acceptMeasurements;

  /**
   * Creates a new Limelight hardware layer.
   *
   * @param name the name of the limelight
   */
  public VisionIOLimelight(
      String name,
      DoubleSupplier gyroAngleSupplier,
      DoubleSupplier gyroAngleRateSupplier,
      boolean acceptMeasurements,
      boolean isLimelight4) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.gyroAngleSupplier = gyroAngleSupplier;
    this.gyroAngleRateSupplier = gyroAngleRateSupplier;
    this.acceptMeasurements = acceptMeasurements;
    this.isLimelight4 = isLimelight4;

    if(isLimelight4) {
      LimelightHelpers.SetIMUAssistAlpha(name, Constants.IMU_ASSIST_ALPHA);
      LimelightHelpers.SetIMUMode(name, Constants.IMU_MODE_EXTERNAL_SEED);
    }
  }

  public void setLEDMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public void updateInputs(VisionIOInputs inputs) {

    // Assume that the pose hasn't been updated
    inputs.poseUpdated = false;

    inputs.tv = getTV();
    inputs.tx = getTXRaw();
    inputs.ty = getTYRaw();
    inputs.pipeline = (int) getPipeline();
    inputs.tagID = getAprilTagID();

    if (acceptMeasurements == false) {
      return;
    }

    // Get the pose estimate from limelight helpers
    Optional<PoseEstimate> newPoseEstimate;
    // If enabled, get megatag 2 pose
    newPoseEstimate = getMegatag2PoseEst();

    // if the new pose estimate is null or angle rate is greater than 720 degrees
    // per, then don't update further
    if (newPoseEstimate.isEmpty()
        || inputs.tv == 0.0
        || gyroAngleRateSupplier.getAsDouble() > 720.0) return;
    // if the megatag1 pose estimate has less than 2 tags in it, don't update
    // further
    if (!newPoseEstimate.get().isMegaTag2) return;
    if (Math.abs(
            newPoseEstimate
                .get()
                .pose
                .getRotation()
                .minus(Rotation2d.fromDegrees(gyroAngleSupplier.getAsDouble()))
                .getDegrees())
        > 60.0) return;

    PoseEstimate poseEstimate = newPoseEstimate.get();

    inputs.estimatedPose = poseEstimate.pose;
    inputs.timestampSeconds = poseEstimate.timestampSeconds;

    // Fill in target information directly into inputs arrays (zero allocation)
    int targetCount = 0;
    for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
      RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
      // if the pose is outside of the field, then skip to the next point
      Optional<Pose3d> tagPose = Constants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
      if (targetCount >= MAX_TAGS || tagPose.isEmpty()) continue;

      inputs.targetIds[targetCount] = rawFiducial.id;
      inputs.distancesToTargets[targetCount] = rawFiducial.distToRobot;
      inputs.tagPoses[targetCount] = tagPose.get();
      targetCount++;
    }

    inputs.targetCount = targetCount;
    inputs.poseUpdated = true;
  }

  private Optional<PoseEstimate> getMegatag2PoseEst() {
    LimelightHelpers.SetRobotOrientation(
        name, gyroAngleSupplier.getAsDouble(), gyroAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.ofNullable(mt2);
  }

  private Optional<PoseEstimate> getMegatag1PoseEst() {
    LimelightHelpers.SetRobotOrientation(
        name, gyroAngleSupplier.getAsDouble(), gyroAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    return Optional.ofNullable(mt2);
  }

  public int getAprilTagID() {
    return (int) table.getEntry("tid").getInteger(-1);
  }

  public double getTXRaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTYRaw() {
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getTV() {
    return table.getEntry("tv").getDouble(0.0);
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0.0);
  }

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public void takeSnapshot() {
    table.getEntry("snapshot").setNumber(1.0);
  }

  public void resetSnapshot() {
    table.getEntry("snapshot").setNumber(0.0);
  }

// while enabled
  public void enableIMUAssist() {
    if (isLimelight4) {
      LimelightHelpers.SetIMUMode(name, Constants.IMU_MODE_INTERNAL_EXTERNAL_ASSIST);
    }
  }

// call during disabled
  public void seedIMU() {
    if(isLimelight4) {
      LimelightHelpers.SetIMUMode(name, Constants.IMU_MODE_EXTERNAL_SEED);
    }
  }
}
