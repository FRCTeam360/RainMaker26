package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.RobotUtils;
import frc.robot.utils.RobotUtils.ActiveHub;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks hub shift state every robot cycle and exposes display values for the Elastic dashboard.
 *
 * <p>Two separate hub-active computations are maintained:
 *
 * <ul>
 *   <li>{@link #isOurHubActive()} — shooter gate, uses full grace-period logic via {@link
 *       RobotUtils#getActiveHub}
 *   <li>{@link #isHubActiveForDisplay()} — Elastic "Can Score in Hub", turns ON early by TOF and
 *       turns OFF at the raw shift boundary (grace covers in-flight balls)
 * </ul>
 *
 * <p>See {@code docs/core_archictecture/HubShiftMechanics.md} for full design rationale.
 */
public class HubShiftTracker {

  public static final double SHIFT_TIME_SECONDS = 25;
  public static final double TIME_TO_SCORE = 2.0; // TODO set to real value
  public static final double TRANSITION_END_SECONDS_SHOOTING = 130 + TIME_TO_SCORE;
  public static final double SHIFT_1_END_SECONDS_SHOOTING = 105 + TIME_TO_SCORE;
  public static final double SHIFT_2_END_SECONDS_SHOOTING = 80 + TIME_TO_SCORE;
  public static final double SHIFT_3_END_SECONDS_SHOOTING = 55 + TIME_TO_SCORE;
  public static final double ENDGAME_START_SECONDS_SHOOTING = 30 + TIME_TO_SCORE;

  private enum MatchPhase {
    AUTO,
    TRANSITION,
    TELEOP,
    ENDGAME,
    DISABLED
  }

  // Cached outputs — updated every call to update()
  private MatchPhase currentPhase = MatchPhase.DISABLED;
  private int teleopShift = 0;
  private boolean isOurHubActive = true;
  private double timeUntilShootingPhaseChange = 0.0;
  private Boolean weWonAuto = false;
  private RobotUtils.ActiveHub activeHub = RobotUtils.ActiveHub.BOTH;
  private Alliance autoWinner = null;
  private Alliance ourAlliance = null;

  /**
   * Creates a new HubShiftTracker.
   *
   * @param hubShotCalculator the shot calculator used to read the current time-of-flight
   */
  public HubShiftTracker() {}

  // -------------------------------------------------------------------------
  // Public API
  // -------------------------------------------------------------------------
  public void update() {
    double matchTime = DriverStation.getMatchTime();

    if (ourAlliance == null) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        ourAlliance = alliance.get();
      }
    }

    // use temp variables to avoid updating actual variables until new values are known
    double tempTimeUntilShootingPhaseChange = timeUntilShootingPhaseChange;
    ActiveHub tempActiveHub = ActiveHub.BOTH;
    int tempTeleopShift = 0;
    boolean tempIsOurHubActive = true;

    if (!DriverStation.isEnabled()) {
      currentPhase = MatchPhase.DISABLED;
      tempTimeUntilShootingPhaseChange = 0; // we don't need a matchTime when disabled
    } else if (DriverStation.isAutonomous()) {
      currentPhase = MatchPhase.AUTO;
      tempTimeUntilShootingPhaseChange =
          matchTime; // in auto the matchTime is equal to auto phase time
    } else if (DriverStation.isTeleop()) {
      if (autoWinner == null) {
        autoWinner = RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage());
        if (autoWinner != null) {
          weWonAuto = autoWinner == ourAlliance;
        }
      }

      if (matchTime > TRANSITION_END_SECONDS_SHOOTING) {
        currentPhase = MatchPhase.TRANSITION;
        tempTimeUntilShootingPhaseChange = matchTime - TRANSITION_END_SECONDS_SHOOTING;

        if (!weWonAuto) {
          tempTimeUntilShootingPhaseChange += SHIFT_TIME_SECONDS;
        }
      } else if (matchTime <= ENDGAME_START_SECONDS_SHOOTING) {
        currentPhase = MatchPhase.ENDGAME;
        tempTimeUntilShootingPhaseChange = matchTime;
      } else {
        currentPhase = MatchPhase.TELEOP;

        if (matchTime > SHIFT_1_END_SECONDS_SHOOTING) {
          tempTeleopShift = 1;
          tempActiveHub = ActiveHub.AUTOLOSER;
          tempIsOurHubActive = !weWonAuto;
          tempTimeUntilShootingPhaseChange = matchTime - SHIFT_1_END_SECONDS_SHOOTING;
        } else if (matchTime > SHIFT_2_END_SECONDS_SHOOTING) {
          tempTeleopShift = 2;
          tempActiveHub = ActiveHub.AUTOWINNER;
          tempIsOurHubActive = weWonAuto;
          tempTimeUntilShootingPhaseChange = matchTime - SHIFT_2_END_SECONDS_SHOOTING;
        } else if (matchTime > SHIFT_3_END_SECONDS_SHOOTING) {
          tempTeleopShift = 3;
          tempActiveHub = ActiveHub.AUTOLOSER;
          tempIsOurHubActive = !weWonAuto;
          tempTimeUntilShootingPhaseChange = matchTime - SHIFT_3_END_SECONDS_SHOOTING;
        } else {
          tempTeleopShift = 4;
          tempActiveHub = ActiveHub.AUTOWINNER;
          tempIsOurHubActive = weWonAuto;
          if (weWonAuto) {
            tempTimeUntilShootingPhaseChange =
                matchTime; // we can score from 4th shift until end of match
          } else {
            tempTimeUntilShootingPhaseChange = matchTime - ENDGAME_START_SECONDS_SHOOTING;
          }
        }
      }
    }

    // update class member variables
    teleopShift = tempTeleopShift;
    activeHub = tempActiveHub;
    isOurHubActive = tempIsOurHubActive;
    timeUntilShootingPhaseChange = tempTimeUntilShootingPhaseChange;
  }

  /**
   * Publishes all hub shift display values to SmartDashboard and logs them via AdvantageKit. Call
   * after {@link #update()} once per cycle.
   */
  public void log() {
    SmartDashboard.putString("Phase", currentPhase.name());
    SmartDashboard.putNumber("Teleop Shift", teleopShift + 1);
    SmartDashboard.putNumber("Time Left in Phase", timeUntilShootingPhaseChange);
    SmartDashboard.putString("Active Hub", activeHub.toString());
    SmartDashboard.putBoolean("Can Score in Hub", isOurHubActive);

    Logger.recordOutput("HubShift/Phase", currentPhase.name());
    Logger.recordOutput("HubShift/TeleopShift", teleopShift + 1);
    Logger.recordOutput("HubShift/TimeLeftInPhase", timeUntilShootingPhaseChange);
    Logger.recordOutput("HubShift/ActiveHub", activeHub.toString());
    Logger.recordOutput("HubShift/HubActive", isOurHubActive);
  }

  /**
   * Returns whether our alliance's hub is currently active for shooting (shooter gate). Uses the
   * full grace-period logic — balls fired just before the shift boundary can still score.
   *
   * @return true if our hub accepts balls right now
   */
  public boolean isOurHubActive() {
    return isOurHubActive;
  }

  public ActiveHub getActiveHub() {
    return activeHub;
  }
}
