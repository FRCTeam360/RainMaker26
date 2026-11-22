// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import java.util.ArrayList;
import java.util.List;

/**
 * Vision IO layer for a Meta Quest headset running QuestNav.
 *
 * <p>QuestNav publishes field-relative poses over NetworkTables at a high rate. This IO converts
 * Quest poses into robot poses and reports them as {@link PoseObservationType#QUESTNAV}
 * observations. It also owns the Quest-specific tracking and battery alerts; connection alerting is
 * handled by {@link Vision}.
 */
public class VisionIOQuestNav implements VisionIO {
  private static final String NAME = "QuestNav";
  private static final int[] NO_TAG_IDS = new int[0];

  private static final int LOW_BATTERY_THRESHOLD_PERCENT = 20;

  // Placeholder estimate of headset-to-robot latency in seconds. QuestNav does not report a
  // per-frame latency, so the timestamp is pushed back by this fixed offset (same pattern as the
  // Limelight latency adjustment). Tune once real latency is measured.
  private static final double QUEST_LATENCY_SECONDS = 0.05;

  private final gg.questnav.questnav.QuestNav questNav = new gg.questnav.questnav.QuestNav();
  private final Transform3d robotToQuest = Constants.ROBOT_TO_QUEST;

  // Reused every loop to avoid allocating a new List each cycle.
  private final List<PoseObservation> observations = new ArrayList<>();

  private final Alert trackingLostAlert = new Alert("QuestNav tracking lost!", AlertType.kWarning);
  private final Alert lowBatteryAlert = new Alert("QuestNav battery low!", AlertType.kWarning);

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Must be called every loop to process command responses and fire callbacks.
    questNav.commandPeriodic();

    boolean connected = questNav.isConnected();
    boolean tracking = connected && questNav.isTracking();
    int batteryPercent = questNav.getBatteryPercent().orElse(-1);

    trackingLostAlert.set(connected && !tracking);
    lowBatteryAlert.set(
        connected && batteryPercent >= 0 && batteryPercent <= LOW_BATTERY_THRESHOLD_PERCENT);

    inputs.connected = connected;
    inputs.hasTarget = false;
    inputs.pipeline = 0;
    inputs.primaryTagId = -1;
    inputs.tagIds = NO_TAG_IDS;
    inputs.batteryPercent = batteryPercent;
    inputs.deviceLatencyMs = questNav.getLatency();

    observations.clear();
    for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
      if (!frame.isTracking() || !connected) {
        continue;
      }

      Pose3d robotPose = frame.questPose3d().transformBy(robotToQuest.inverse());
      if (!isFinite(robotPose.toPose2d())) {
        continue;
      }

      observations.add(
          new PoseObservation(
              frame.dataTimestamp() - QUEST_LATENCY_SECONDS,
              robotPose,
              0.0,
              0,
              0.0,
              PoseObservationType.QUESTNAV));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }

  @Override
  public void setPose(Pose2d robotPose) {
    questNav.setPose(new Pose3d(robotPose).transformBy(robotToQuest));
  }

  @Override
  public String getName() {
    return NAME;
  }

  private static boolean isFinite(Pose2d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
}
