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
public class HubShiftTrackerSimplified {

  public static final double SHIFT_TIME_SECONDS = 25;
  public static final double TIME_TO_SCORE_SECONDS = 2.0; // TODO set to real value
  public static final double GRACE_PERIOD_SECONDS = 3.0;
  public static final double TRANSITION_END_SECONDS_SHOOTING = 130 + TIME_TO_SCORE_SECONDS - GRACE_PERIOD_SECONDS;
  public static final double SHIFT_1_START_SECONDS_SHOOTING = 130 + TIME_TO_SCORE_SECONDS;
  public static final double SHIFT_1_END_SECONDS_SHOOTING = 105 + TIME_TO_SCORE_SECONDS - GRACE_PERIOD_SECONDS;
  public static final double SHIFT_2_START_SECONDS_SHOOTING = 105 + TIME_TO_SCORE_SECONDS;
  public static final double SHIFT_2_END_SECONDS_SHOOTING = 80 + TIME_TO_SCORE_SECONDS - GRACE_PERIOD_SECONDS;
  public static final double SHIFT_3_START_SECONDS_SHOOTING = 80 + TIME_TO_SCORE_SECONDS;
  public static final double SHIFT_3_END_SECONDS_SHOOTING = 55 + TIME_TO_SCORE_SECONDS - GRACE_PERIOD_SECONDS;
  public static final double ENDGAME_START_SECONDS_SHOOTING = 30 + TIME_TO_SCORE_SECONDS;
  public static final double ENDGAME_END_SECONDS_SHOOTING = 30 + TIME_TO_SCORE_SECONDS - GRACE_PERIOD_SECONDS;

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
   * Creates a new HubShiftTrackerSimplified.
   *
   * @param hubShotCalculator the shot calculator used to read the current time-of-flight
   */
  public HubShiftTrackerSimplified() {}

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

    if (!DriverStation.isEnabled()) {
      currentPhase = MatchPhase.DISABLED;
      timeUntilShootingPhaseChange = 0; // we don't need a matchTime when disabled
      activeHub = ActiveHub.BOTH;
      teleopShift = 0;
      isOurHubActive = false;
    } else if (DriverStation.isAutonomous()) {
      currentPhase = MatchPhase.AUTO;
      timeUntilShootingPhaseChange = matchTime; // in auto the matchTime is equal to auto phase time
      activeHub = ActiveHub.BOTH;
      teleopShift = 0;
      isOurHubActive = true;
    } else if (DriverStation.isTeleop()) {
      if (autoWinner == null) {
        autoWinner = RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage());
        if (autoWinner != null) {
          weWonAuto = autoWinner == ourAlliance;
        }
      }

      ShiftValues shiftValues = getTeleopShiftValues(matchTime, weWonAuto);
      currentPhase = shiftValues.currentPhase;
      teleopShift = shiftValues.teleopShiftNumber;
      activeHub = shiftValues.activeHub;
      isOurHubActive = shiftValues.isOurHubActive;
      timeUntilShootingPhaseChange = shiftValues.timeUntilShootingPhaseChange;
    }
  }

  public static ShiftValues getTeleopShiftValues(
    double matchTime,
    boolean weWonAuto) {
      ShiftValues values = new ShiftValues(0, MatchPhase.TELEOP, ActiveHub.BOTH, true, matchTime);
      
      if (matchTime > TRANSITION_END_SECONDS_SHOOTING) {
        values.currentPhase = MatchPhase.TRANSITION;
        values.timeUntilShootingPhaseChange = matchTime - TRANSITION_END_SECONDS_SHOOTING;

        if (!weWonAuto) {
          values.timeUntilShootingPhaseChange += SHIFT_TIME_SECONDS;
        }
      } else if (matchTime <= ENDGAME_START_SECONDS_SHOOTING) {
        values.currentPhase = MatchPhase.ENDGAME;
        values.timeUntilShootingPhaseChange = matchTime;
      } else {
        values.currentPhase = MatchPhase.TELEOP;

        if (matchTime > SHIFT_1_END_SECONDS_SHOOTING) {
          values.teleopShiftNumber = 1;
          values.activeHub = ActiveHub.AUTOLOSER;
          values.isOurHubActive = !weWonAuto;
          values.timeUntilShootingPhaseChange = matchTime - SHIFT_1_END_SECONDS_SHOOTING;
        } else if (matchTime > SHIFT_2_END_SECONDS_SHOOTING) {
          values.teleopShiftNumber = 2;
          values.activeHub = ActiveHub.AUTOWINNER;
          values.isOurHubActive = weWonAuto;
          values.timeUntilShootingPhaseChange = matchTime - SHIFT_2_END_SECONDS_SHOOTING;
        } else if (matchTime > SHIFT_3_END_SECONDS_SHOOTING) {
          values.teleopShiftNumber = 3;
          values.activeHub = ActiveHub.AUTOLOSER;
          values.isOurHubActive = !weWonAuto;
          values.timeUntilShootingPhaseChange = matchTime - SHIFT_3_END_SECONDS_SHOOTING;
        } else {
          values.teleopShiftNumber = 4;
          values.activeHub = ActiveHub.AUTOWINNER;
          values.isOurHubActive = weWonAuto;
          if (weWonAuto) {
            values.timeUntilShootingPhaseChange =
                matchTime; // we can score from 4th shift until end of match
          } else {
            values.timeUntilShootingPhaseChange = matchTime - ENDGAME_START_SECONDS_SHOOTING;
          }
        }
      }

      return values;
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
