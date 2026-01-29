// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class VisionIOWB implements VisionIO {
  private final NetworkTable table;
  private final String name;
  private final DoubleSupplier gyroAngleSupplier;
  private final DoubleSupplier gyroAngleRateSupplier;

  private boolean acceptMeasurements = false;

  // private RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

  /** Creates a new VisionIOWB. */
  public VisionIOWB(
      String name,
      DoubleSupplier gyroAngleSupplier,
      DoubleSupplier gyroAngleRateSupplier,
      boolean acceptMeasurements) {

    table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.gyroAngleSupplier = gyroAngleSupplier;
    this.gyroAngleRateSupplier = gyroAngleRateSupplier;
    this.acceptMeasurements = acceptMeasurements;
  }

  // last year's code had two constructers, the second one had no accpeted
  // measurements, is that necessary?

  public void setLEDMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.poseUpdated = false;

    inputs.tv = getTVRaw();
    inputs.tx = getTXRaw();
    inputs.ty = getTYRaw();
    inputs.pipeline = (int) getPipeline();
    inputs.tagID = getAprilTagID();

    if (acceptMeasurements == false) {
      return;
    }

    Optional<PoseEstimator> newPoseEstimate;

    // newPoseEstimate = getMegatag2PoseEst();

    // if (newPoseEstimate.isEmpty())
    //   return;

    // if (inputs.tv == 0.0 || newPoseEstimate.isEmpty() || gyroAngleRateSupplier.getAsDouble() >
    // 720.0)
    //   return;

    // if (!newPoseEstimate.get().isMegaTag2)
    //   return;

    // if (Math.abs(
    //
    // newPoseEstimate.get().pose.getRotation().minus(Rotation2d.fromDegrees(gyroAngleSupplier.getAsDouble()))
    //         .getDegrees()) > 60.0)
    //   return;

    // PoseEstimator poseEstimate = newPoseEstimate.get();

    // inputs.estimatedPose = poseEstimate.pose;
    // inputs.timestampSeconds = poseEstimate.timestampSeconds;
    // int[] targetIds = new int[poseEstimate.rawFiducials.length];
    // double[] distancesToTargets = new double[poseEstimate.rawFiducials.length];
    // Pose3d[] tagPoses = new Pose3d[poseEstimate.rawFiducials.length];
    // // for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
    //   RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
    //   // if the pose is outside of the field, then skip to the next point
    //   Optional<Pose3d> tagPose = Constants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
    //   if (tagPose.isEmpty())
    //     continue;

    //   targetIds[i] = rawFiducial.id;
    //   distancesToTargets[i] = rawFiducial.distToRobot;
    //   tagPoses[i] = tagPose.get();
    // }
    // inputs.targetIds = targetIds;
    // inputs.distancesToTargets = distancesToTargets;
    // inputs.tagPoses = tagPoses;
    // inputs.poseUpdated = true;

  }

  // private Optional<PoseEstimate> getMegaTag2PoseEst() {
  //   LimelightHelpers.setRobotOrientation(
  //       name, gyroAngleSupplier.getAsDouble(), gyroAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);
  //   PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  //   return Optional.ofNullable(mt2);
  // }

  // private Optional<PoseEstimate> getMegatag1PoseEst() {
  //   LimelightHelpers.SetRobotOrientation(
  //       name, gyroAngleSupplier.getAsDouble(), gryoAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);
  //   PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  //   return Optional.ofNullable(mt2);
  // }

  public int getAprilTagID() {
    return (int) table.getEntry("tid").getInteger(0);
  }

  public double getTXRaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTYRaw() {
    return table.getEntry("ty").getDouble(0);
  }

  public double getTVRaw() {
    return table.getEntry("tv").getDouble(0);
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public void takeSnapshot() {
    table.getEntry("snapshot").setNumber(1);
  }

  public void resetSnapshot() {
    table.getEntry("snapshot").setNumber(0);
  }
}
