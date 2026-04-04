package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.RobotUtils;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks match phase and time remaining for the Elastic dashboard.
 *
 * <p>Publishes "Time Left in Phase" (countdown to next hub shift / phase boundary) each cycle.
 */
public class HubShiftTracker {

  private enum MatchPhase {
    AUTO,
    TRANSITION,
    TELEOP,
    ENDGAME,
    DISABLED
  }

  // Cached outputs — updated every call to update()
  private MatchPhase currentPhase = MatchPhase.DISABLED;
  private double primaryTimeLeft = 0.0;

  /** Creates a new HubShiftTracker. */
  public HubShiftTracker() {}

  // -------------------------------------------------------------------------
  // Public API
  // -------------------------------------------------------------------------

  /**
   * Updates all cached state. Must be called once per robot cycle, before reading any getters.
   * Reads DriverStation directly for match time.
   */
  public void update() {
    double matchTime = DriverStation.getMatchTime();

    // --- Phase detection ---
    if (!DriverStation.isEnabled()) {
      currentPhase = MatchPhase.DISABLED;
    } else if (DriverStation.isAutonomous()) {
      currentPhase = MatchPhase.AUTO;
    } else if (DriverStation.isTeleop()) {
      if (matchTime > RobotUtils.TRANSITION_END_SECONDS) {
        currentPhase = MatchPhase.TRANSITION;
      } else if (matchTime <= RobotUtils.ENDGAME_START_SECONDS) {
        currentPhase = MatchPhase.ENDGAME;
      } else {
        currentPhase = MatchPhase.TELEOP;
      }
    }

    // --- Primary countdown: time until the next phase/shift boundary ---
    primaryTimeLeft = computeTimeLeftInPhase(matchTime);
  }

  /**
   * Publishes phase and time values to SmartDashboard and AdvantageKit. Call after {@link
   * #update()} once per cycle.
   */
  public void log() {
    SmartDashboard.putString("Phase", currentPhase.name());
    SmartDashboard.putNumber("Time Left in Phase", primaryTimeLeft);

    Logger.recordOutput("HubShift/Phase", currentPhase.name());
    Logger.recordOutput("HubShift/TimeLeftInPhase", primaryTimeLeft);
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  private double computeTimeLeftInPhase(double matchTime) {
    switch (currentPhase) {
      case AUTO:
        return matchTime;
      case TRANSITION:
        return matchTime - RobotUtils.TRANSITION_END_SECONDS;
      case TELEOP:
        return getTimeUntilNextShiftBoundary(matchTime);
      case ENDGAME:
        return matchTime;
      default:
        return 0.0;
    }
  }

  private double getTimeUntilNextShiftBoundary(double matchTime) {
    if (matchTime > RobotUtils.SHIFT_1_END_SECONDS) {
      return matchTime - RobotUtils.SHIFT_1_END_SECONDS;
    } else if (matchTime > RobotUtils.SHIFT_2_END_SECONDS) {
      return matchTime - RobotUtils.SHIFT_2_END_SECONDS;
    } else if (matchTime > RobotUtils.SHIFT_3_END_SECONDS) {
      return matchTime - RobotUtils.SHIFT_3_END_SECONDS;
    } else if (matchTime > RobotUtils.ENDGAME_START_SECONDS) {
      return matchTime - RobotUtils.ENDGAME_START_SECONDS;
    } else {
      return Math.max(0, matchTime);
    }
  }
}
