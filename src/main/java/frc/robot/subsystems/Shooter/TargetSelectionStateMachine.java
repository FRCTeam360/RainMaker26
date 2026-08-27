package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    PASS, // Force pass (manual override)
    AUTO // Auto-select based on alliance zone position
  }

  public enum TargetInternalStates {
    SCORING,
    PASSING
  }

  // Dependencies
  private final ShotCalculator hubShotCalculator;
  private final ShotCalculator passCalculator;
  private final Supplier<Pose2d> robotPoseSupplier;

  // State variables
  private TargetWantedStates wantedState = TargetWantedStates.AUTO;
  private TargetInternalStates currentState = TargetInternalStates.SCORING;
  private TargetInternalStates previousState = TargetInternalStates.SCORING;

  /**
   * Creates a new TargetSelectionStateMachine.
   *
   * @param hubShotCalculator the shot calculator for hub shots
   * @param passCalculator the shot calculator for passes
   * @param robotPoseSupplier supplier for the robot's current pose
   */
  public TargetSelectionStateMachine(
      ShotCalculator hubShotCalculator,
      ShotCalculator passCalculator,
      Supplier<Pose2d> robotPoseSupplier) {
    this.hubShotCalculator = hubShotCalculator;
    this.passCalculator = passCalculator;
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
    if (currentState == TargetInternalStates.PASSING) {
      return passCalculator;
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
        currentState = TargetInternalStates.SCORING;
        break;
      case PASS:
        currentState = TargetInternalStates.PASSING;
        break;
      case AUTO:
      default:
        if (PositionUtils.isInAllianceZone(robotPoseSupplier.get())) {
          currentState = TargetInternalStates.SCORING;
        } else {
          currentState = TargetInternalStates.PASSING;
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
    SmartDashboard.putString(
        "Superstructure/TargetSelection/CurrentState", currentState.toString());
    Logger.recordOutput("Superstructure/TargetSelection/PreviousState", previousState);
  }
}
