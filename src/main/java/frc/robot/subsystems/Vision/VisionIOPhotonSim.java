// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionSimConstants.VisionPhotonSim.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision simulation implementation of VisionIO.
 *
 * <p>This class simulates AprilTag detection using PhotonVision's simulation framework. It requires
 * a robot pose supplier to know where the simulated camera is on the field.
 */
public class VisionIOPhotonSim implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final Supplier<Pose2d> robotPoseSupplier;

  // Simulation objects
  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;

  // Track pipeline (not used in sim but required by interface)
  private int currentPipeline = 0;

  /**
   * Creates a new VisionIOPhotonSim.
   *
   * @param robotPoseSupplier Supplier for the simulated robot pose (from drivetrain simulation).
   *     This should be the TRUE simulated pose, not the estimated pose.
   */
  public VisionIOPhotonSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    // Create the PhotonCamera with the configured name
    camera = new PhotonCamera(kCameraName);

    // Create the pose estimator with the tag layout and robot-to-camera transform
    photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    // --- Simulation Setup ---

    // Create the vision system simulation which handles cameras and targets on the field
    visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this simulated field
    visionSim.addAprilTags(kTagLayout);

    // Create simulated camera properties to mimic actual camera behavior
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90)); // Resolution and FOV
    cameraProp.setCalibError(0.15, 0.05); // Reduced calibration noise
    cameraProp.setFPS(30); // Higher FPS to match robot loop better
    cameraProp.setAvgLatencyMs(30); // Reduced average latency
    cameraProp.setLatencyStdDevMs(5); // Reduced latency variation

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Set maximum sight range - tags beyond this distance won't be detected
    cameraSim.setMaxSightRange(5.0); // 5 meters max detection distance

    // Add the simulated camera to view the targets on this simulated field
    visionSim.addCamera(cameraSim, kRobotToCam);

    // Enable wireframe drawing for visualization (resource-intensive but helpful for debugging)
    cameraSim.enableDrawWireframe(false);
  }

  private void updateInputsWhenNoTargets(VisionIOInputs inputs) {
    inputs.tv = 0.0;
    inputs.tx = 0.0;
    inputs.ty = 0.0;
    inputs.tagID = -1;
    inputs.targetCount = 0;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update the vision simulation with the current robot pose
    Pose2d robotPose = robotPoseSupplier.get();
    visionSim.update(robotPose);

    // Assume pose hasn't been updated until we confirm otherwise
    inputs.poseUpdated = false;
    inputs.pipeline = currentPipeline;

    // Process all unread camera results
    var results = camera.getAllUnreadResults();

    // Check if we have new results OR get the latest result even if it's old
    var latestResult =
        results.isEmpty() ? camera.getLatestResult() : results.get(results.size() - 1);

    // If we have no result at all (not even a cached one)
    if (latestResult == null) {
      updateInputsWhenNoTargets(inputs);
      return;
    }

    List<PhotonTrackedTarget> targets = latestResult.getTargets();

    // If no targets detected
    if (targets.isEmpty()) {
      updateInputsWhenNoTargets(inputs);
      return;
    }

    // We have at least one target
    inputs.tv = 1.0;

    // Get the best target for tx/ty
    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
    if (bestTarget != null) {
      inputs.tx = bestTarget.getYaw();
      inputs.ty = bestTarget.getPitch();
      inputs.tagID = bestTarget.getFiducialId();
    }

    // Try to estimate robot pose using multi-tag first, then fall back to single tag
    Optional<EstimatedRobotPose> visionEst =
        photonEstimator.estimateCoprocMultiTagPose(latestResult);
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(latestResult);
    }

    // If we got a pose estimate, fill in the inputs
    if (visionEst.isPresent()) {
      EstimatedRobotPose est = visionEst.get();

      inputs.estimatedPose = est.estimatedPose.toPose2d();
      inputs.timestampSeconds = est.timestampSeconds;
      inputs.poseUpdated = true;

      // Fill in target information directly into inputs arrays (zero allocation)
      int targetCount = 0;

      for (PhotonTrackedTarget target : targets) {
        int tagId = target.getFiducialId();
        Optional<Pose3d> tagPose = kTagLayout.getTagPose(tagId);

        if (targetCount >= MAX_TAGS || tagPose.isEmpty()) continue;
        inputs.targetIds[targetCount] = tagId;

        // Calculate distance from robot to tag
        double distance =
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        inputs.distancesToTargets[targetCount] = distance;

        inputs.tagPoses[targetCount] = tagPose.get();
        targetCount++;
      }

      inputs.targetCount = targetCount;

      // Update the debug field visualization
      getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d());
    } else {
      // Clear the visualization if no pose
      getSimDebugField().getObject("VisionEstimation").setPoses();
    }
  }

  @Override
  public void setLEDMode(int mode) {
    // LEDs are not simulated - no-op
  }

  @Override
  public void setPipeline(int pipeline) {
    // Store the pipeline value (pipelines are not simulated)
    currentPipeline = pipeline;
  }

  @Override
  public void takeSnapshot() {
    // Snapshots are not simulated - no-op
  }

  @Override
  public void resetSnapshot() {
    // Snapshots are not simulated - no-op
  }

  // ----- Simulation-specific methods -----

  /**
   * Resets the pose history of the robot in the vision system simulation. Call this when resetting
   * the robot pose in simulation.
   *
   * @param pose The pose to reset to
   */
  public void resetSimPose(Pose2d pose) {
    visionSim.resetRobotPose(pose);
  }

  public void captureRewind() {}

  /**
   * Gets the Field2d for visualizing the simulation.
   *
   * @return The simulation debug field
   */
  public Field2d getSimDebugField() {
    return visionSim.getDebugField();
  }

  /**
   * Gets the underlying VisionSystemSim for advanced configuration.
   *
   * @return The vision system simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Gets the underlying PhotonCameraSim for advanced configuration.
   *
   * @return The camera simulation
   */
  public PhotonCameraSim getCameraSim() {
    return cameraSim;
  }
}
