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
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Abstract base class for Limelight vision IO layers. Contains all shared NetworkTables reads, pose
 * filtering, and MegaTag2 logic.
 */
public abstract class VisionIOLimelightBase implements VisionIO {
  private static final PoseObservation[] NO_OBSERVATIONS = new PoseObservation[0];
  private static final int[] NO_TAG_IDS = new int[0];

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
  @Override
  public String getName() {
    return name;
  }

  private void pollCameraPose() {
    if (cameraPoseResolved) return;
    if (LimelightHelpers.getHeartbeat(name) == 0) return;
    Pose3d pose = LimelightHelpers.getCameraPose3d_RobotSpace(name);
    if (pose.getTranslation().getNorm() == 0.0 && pose.getRotation().getAngle() == 0.0) return;

    cameraPoseRobotSpace = pose;
    cameraPoseResolved = true;
  }

  private void updateObservedTagOrientation(VisionIOInputs inputs) {
    Rotation3d rotation = LimelightHelpers.getTargetPose3d_RobotSpace(name).getRotation();
    inputs.nearestTagObservedRollDeg = Math.toDegrees(rotation.getX());
    inputs.nearestTagObservedPitchDeg = Math.toDegrees(rotation.getY());
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
    inputs.cameraPose = cameraPoseRobotSpace;
    inputs.connected = LimelightHelpers.getHeartbeat(name) != 0;
    // Set robot orientation for MegaTag2 (flushed by postSchedulerUpdate)
    LimelightHelpers.SetRobotOrientation_NoFlush(
        name, gyroAngleSupplier.getAsDouble(), gyroAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);

    // Start with no observations until we confirm one is valid
    inputs.poseObservations = NO_OBSERVATIONS;
    inputs.tagIds = NO_TAG_IDS;

    inputs.hasTarget = getTV() != 0.0;
    inputs.txDeg = getTXRaw();
    inputs.tyDeg = getTYRaw();
    inputs.pipeline = (int) getPipeline();
    inputs.primaryTagId = getAprilTagID();
    inputs.hasIMU = hasIMU();

    if (!acceptMeasurements) {
      return;
    }

    // Get the pose estimate from limelight helpers
    Optional<PoseEstimate> newPoseEstimate = getMegatag2PoseEst();

    // if the new pose estimate is null or angle rate is greater than 720 degrees
    // per, then don't update further
    if (newPoseEstimate.isEmpty()
        || !inputs.hasTarget
        || Math.abs(gyroAngleRateSupplier.getAsDouble()) > 720.0) return;
    // Only MegaTag2 poses are accepted
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

    // Collect observed tags and the closest one (used to scale measurement trust)
    int[] tagIds = new int[poseEstimate.rawFiducials.length];
    int tagCount = 0;
    double closestTagDistance = Double.MAX_VALUE;
    for (RawFiducial rawFiducial : poseEstimate.rawFiducials) {
      Optional<Pose3d> tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
      if (tagPose.isEmpty()) continue;

      tagIds[tagCount++] = rawFiducial.id;
      closestTagDistance = Math.min(closestTagDistance, rawFiducial.distToRobot);
    }
    if (tagCount == 0) {
      return;
    }
    inputs.tagIds = Arrays.copyOf(tagIds, tagCount);

    double ambiguity = poseEstimate.tagCount == 1 ? poseEstimate.rawFiducials[0].ambiguity : 0.0;

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              poseEstimate.timestampSeconds,
              new Pose3d(poseEstimate.pose),
              ambiguity,
              poseEstimate.tagCount,
              closestTagDistance,
              PoseObservationType.MEGATAG_2)
        };
    updateObservedTagOrientation(inputs);
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
