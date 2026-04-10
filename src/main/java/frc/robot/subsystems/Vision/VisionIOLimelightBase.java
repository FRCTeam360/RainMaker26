// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Abstract base class for Limelight vision IO layers. Contains all shared NetworkTables reads, pose
 * filtering, and MegaTag2 logic.
 */
public abstract class VisionIOLimelightBase implements VisionIO {
  private final NetworkTable table;
  private final String name;
  protected final DoubleSupplier gyroAngleSupplier;
  protected final DoubleSupplier gyroAngleRateSupplier;

  private final boolean acceptMeasurements;

  private Pose3d cameraPoseRobotSpace = new Pose3d();

  private boolean cameraPoseResolved = false;

  /**
   * Creates a new Limelight hardware layer.
   *
   * @param name the NetworkTables name of the Limelight
   * @param gyroAngleSupplier supplies the robot's gyro angle in degrees
   * @param gyroAngleRateSupplier supplies the robot's gyro angular rate in degrees per second
   * @param acceptMeasurements whether to process pose estimates from this Limelight
   */
  protected VisionIOLimelightBase(
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

  /** Returns the NetworkTables name of this Limelight. */
  protected String getName() {
    return name;
  }

  public void pollCameraPose() {
    if (cameraPoseResolved) return;
    if (LimelightHelpers.getHeartbeat(name) == 0) return;
    Pose3d pose = LimelightHelpers.getCameraPose3d_RobotSpace(name);
    if (pose.getTranslation().getNorm() == 0.0 && pose.getRotation().getAngle() == 0.0) return;

    cameraPoseRobotSpace = pose;
    cameraPoseResolved = true;
  }

  private void updateNearestTagOrientation(VisionIOInputs inputs, PoseEstimate poseEstimate) {
    int nearestTagID = -1;
    double nearestDistance = Double.MAX_VALUE;
    for (RawFiducial fiducial : poseEstimate.rawFiducials) {
      if (fiducial.distToRobot < nearestDistance) {
        nearestDistance = fiducial.distToRobot;
        nearestTagID = fiducial.id;
      }
    }

    if (nearestTagID == -1) return;

    LimelightResults results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return;

    for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
      if ((int) target.fiducialID == nearestTagID) {
        Pose3d tagPoseInRobotSpace = target.getTargetPose_RobotSpace();
        Rotation3d rotation = tagPoseInRobotSpace.getRotation();
        inputs.nearestTagObservedPitchDeg = Math.toDegrees(rotation.getY());
        inputs.nearestTagObservedRollDeg = Math.toDegrees(rotation.getX());
      }
    }
  }

  @Override
  public void setLEDMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  protected boolean hasIMU() {
    return false;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    pollCameraPose();
    // Set robot orientation for MegaTag2 (flushed by postSchedulerUpdate)
    LimelightHelpers.SetRobotOrientation_NoFlush(
        name, gyroAngleSupplier.getAsDouble(), gyroAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);

    // Assume that the pose hasn't been updated
    inputs.poseUpdated = false;

    inputs.tv = getTV();
    inputs.tx = getTXRaw();
    inputs.ty = getTYRaw();
    inputs.pipeline = (int) getPipeline();
    inputs.tagID = getAprilTagID();
    inputs.hasIMU = hasIMU();

    if (!acceptMeasurements) {
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
        || Math.abs(gyroAngleRateSupplier.getAsDouble()) > 720.0) return;
    // if the megatag1 pose estimate has less than 2 tags in it, don't update
    // further
    if (!newPoseEstimate.get().isMegaTag2) return;
    if (newPoseEstimate.get().tagCount == 0) return;
    if (Math.abs(
            newPoseEstimate
                .get()
                .pose
                .getRotation()
                .minus(Rotation2d.fromDegrees(gyroAngleSupplier.getAsDouble()))
                .getDegrees())
        > 120.0) return;

    PoseEstimate poseEstimate = newPoseEstimate.get();

    inputs.estimatedPose = poseEstimate.pose;
    inputs.timestampSeconds = poseEstimate.timestampSeconds;

    // Fill in target information directly into inputs arrays (zero allocation)
    int targetCount = 0;
    for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
      RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
      // if the pose is outside of the field, then skip to the next point
      Optional<Pose3d> tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
      if (targetCount >= MAX_TAGS || tagPose.isEmpty()) continue;

      inputs.targetIds[targetCount] = rawFiducial.id;
      inputs.distancesToTargets[targetCount] = rawFiducial.distToRobot;
      inputs.tagPoses[targetCount] = tagPose.get();
      targetCount++;
    }

    inputs.targetCount = targetCount;
    inputs.poseUpdated = true;
    updateNearestTagOrientation(inputs, poseEstimate);
  }

  private Optional<PoseEstimate> getMegatag2PoseEst() {
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.ofNullable(mt2);
  }

  private int getAprilTagID() {
    return (int) table.getEntry("tid").getInteger(-1);
  }

  private double getTXRaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  private double getTYRaw() {
    return table.getEntry("ty").getDouble(0.0);
  }

  private double getTV() {
    return table.getEntry("tv").getDouble(0.0);
  }

  private double getPipeline() {
    return table.getEntry("getpipe").getDouble(0.0);
  }

  @Override
  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void takeSnapshot() {
    table.getEntry("snapshot").setNumber(1.0);
  }

  @Override
  public void resetSnapshot() {
    table.getEntry("snapshot").setNumber(0.0);
  }
}
