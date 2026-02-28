package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.PositionUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages target selection state, deciding whether to shoot at the hub or outpost based on manual
 * overrides or automatic alliance zone detection.
 */
public class TargetSelectionStateMachine {
  // Enums
  public enum TargetWantedStates {
    HUB, // Force hub (manual override)
    OUTPOST, // Force outpost (manual override)
    AUTO // Auto-select based on alliance zone position
  }

  public enum TargetInternalStates {
    AT_HUB,
    AT_OUTPOST
  }

  // Dependencies
  private final ShotCalculator hubShotCalculator;
  private final ShotCalculator outpostPassCalculator;
  private final Supplier<Pose2d> robotPoseSupplier;

  // State variables
  private TargetWantedStates wantedState = TargetWantedStates.AUTO;
  private TargetInternalStates currentState = TargetInternalStates.AT_HUB;
  private TargetInternalStates previousState = TargetInternalStates.AT_HUB;

  /**
   * Creates a new TargetSelectionStateMachine.
   *
   * @param hubShotCalculator the shot calculator for hub shots
   * @param outpostPassCalculator the shot calculator for outpost passes
   * @param robotPoseSupplier supplier for the robot's current pose
   */
  public TargetSelectionStateMachine(
      ShotCalculator hubShotCalculator,
      ShotCalculator outpostPassCalculator,
      Supplier<Pose2d> robotPoseSupplier) {
    this.hubShotCalculator = hubShotCalculator;
    this.outpostPassCalculator = outpostPassCalculator;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  /**
   * Sets the wanted target state.
   *
   * @param state the desired target state
   */
  public void setWantedState(TargetWantedStates state) {
    wantedState = state;
  }

  /** Returns the current resolved target state. */
  public TargetInternalStates getState() {
    return currentState;
  }

  /**
   * Returns the shot calculator for the currently resolved target.
   *
   * @return the outpost calculator when targeting outpost, hub calculator otherwise
   */
  public ShotCalculator getActiveCalculator() {
    if (currentState == TargetInternalStates.AT_OUTPOST) {
      return outpostPassCalculator;
    }
    return hubShotCalculator;
  }

  /**
   * Resolves the wanted state into the current target state. For AUTO mode, uses alliance zone
   * position to determine the target. Should be called inline before reading the resolved state to
   * avoid one-cycle lag.
   */
  public void update() {
    previousState = currentState;

    switch (wantedState) {
      case HUB:
        currentState = TargetInternalStates.AT_HUB;
        break;
      case OUTPOST:
        currentState = TargetInternalStates.AT_OUTPOST;
        break;
      case AUTO:
      default:
        if (PositionUtils.isInAllianceZone(robotPoseSupplier.get())) {
          currentState = TargetInternalStates.AT_HUB;
        } else {
          currentState = TargetInternalStates.AT_OUTPOST;
        }
        break;
    }
  }

  /** No-op for now. Pattern placeholder matching ShooterStateMachine. */
  public void apply() {}

  /** Logs the wanted, current, and previous target selection states. */
  public void log() {
    Logger.recordOutput("Superstructure/TargetSelection/WantedState", wantedState);
    Logger.recordOutput("Superstructure/TargetSelection/CurrentState", currentState);
    Logger.recordOutput("Superstructure/TargetSelection/PreviousState", previousState);
  }
}
