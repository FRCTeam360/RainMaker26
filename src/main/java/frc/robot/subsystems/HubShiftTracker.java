package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParams;
import frc.robot.utils.RobotUtils;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks hub shift state every robot cycle and exposes display values for the Elastic dashboard.
 *
 * <p>Two separate hub-active computations are maintained:
 *
 * <ul>
 *   <li>{@link #isHubActive()} — shooter gate, uses full grace-period logic via {@link
 *       RobotUtils#getActiveHub}
 *   <li>{@link #isHubActiveForDisplay()} — Elastic "Can Score in Hub", turns ON early by TOF and
 *       turns OFF at the raw shift boundary (grace covers in-flight balls)
 * </ul>
 *
 * <p>See {@code docs/core_archictecture/HubShiftMechanics.md} for full design rationale.
 */
public class HubShiftTracker {

  // Extra time after the ball enters the hub before it reaches the scoring sensor.
  // Added to the TOF so we don't shoot into a hub that's about to go inactive before the ball
  // actually registers as scored.
  private static final double BALL_TO_SENSOR_DELAY_SECONDS = 0.5;

  private enum MatchPhase {
    AUTO,
    TRANSITION,
    TELEOP,
    ENDGAME,
    DISABLED
  }

  private final ShotCalculator hubShotCalculator;

  // Cached outputs — updated every call to update()
  private MatchPhase currentPhase = MatchPhase.DISABLED;
  private int teleopShift = 0;
  private boolean hubActive = true;
  private boolean hubActiveForDisplay = true;
  private double primaryTimeLeft = 0.0;
  private double cachedTimeOfFlight = 0.0;
  private RobotUtils.ActiveHub activeHub = RobotUtils.ActiveHub.BOTH;

  /**
   * Creates a new HubShiftTracker.
   *
   * @param hubShotCalculator the shot calculator used to read the current time-of-flight
   */
  public HubShiftTracker(ShotCalculator hubShotCalculator) {
    this.hubShotCalculator = hubShotCalculator;
  }

  // -------------------------------------------------------------------------
  // Public API
  // -------------------------------------------------------------------------

  /**
   * Updates all cached state. Must be called once per robot cycle, before reading any getters.
   * Reads DriverStation directly for match time, alliance, and game-specific message.
   */
  public void update() {
    // --- TOF ---
    // Only use the calculated TOF when the shot is valid; fall back to the minimum so an
    // out-of-range pose doesn't produce a huge TOF that shifts gameTime into the endgame bucket.
    ShootingParams hubShot = hubShotCalculator.calculateShot();
    cachedTimeOfFlight =
        hubShot.isValid() ? hubShot.timeOfFlight() : hubShotCalculator.getMinTimeOfFlightSecs();

    double matchTimeRaw = DriverStation.getMatchTime();
    double effectiveTof = getEffectiveTof();
    double matchTimeAdjusted = matchTimeRaw - effectiveTof;

    // --- Phase detection (uses adjusted time so phase flips TOF-early) ---
    if (!DriverStation.isEnabled()) {
      currentPhase = MatchPhase.DISABLED;
    } else if (DriverStation.isAutonomous()) {
      currentPhase = MatchPhase.AUTO;
    } else if (DriverStation.isTeleop()) {
      if (matchTimeAdjusted > RobotUtils.TRANSITION_END_SECONDS) {
        currentPhase = MatchPhase.TRANSITION;
      } else if (matchTimeAdjusted <= RobotUtils.ENDGAME_START_SECONDS) {
        currentPhase = MatchPhase.ENDGAME;
      } else {
        currentPhase = MatchPhase.TELEOP;
      }
    }

    // --- Teleop shift number (1-indexed, 0 outside teleop) ---
    if (currentPhase == MatchPhase.TELEOP) {
      if (matchTimeAdjusted > RobotUtils.SHIFT_1_END_SECONDS) teleopShift = 1;
      else if (matchTimeAdjusted > RobotUtils.SHIFT_2_END_SECONDS) teleopShift = 2;
      else if (matchTimeAdjusted > RobotUtils.SHIFT_3_END_SECONDS) teleopShift = 3;
      else teleopShift = 4;
    } else {
      teleopShift = 0;
    }

    // --- Shooter gate (with grace period) ---
    activeHub = RobotUtils.getActiveHub(matchTimeRaw, DriverStation.isTeleop(), effectiveTof);
    Alliance autoWinner = RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage());
    Optional<Alliance> alliance = DriverStation.getAlliance();

    hubActive = RobotUtils.isHubActiveForAlliance(alliance, autoWinner, activeHub);

    // --- Display hub active (asymmetric TOF) ---
    // Turns ON early (adjusted time) so driver can shoot as soon as ball will land while active.
    // Turns OFF at raw boundary — 2 s grace covers any in-flight balls fired before the boundary.
    hubActiveForDisplay =
        RobotUtils.isHubActiveForAlliance(
                alliance, autoWinner, getDisplayActiveHub(matchTimeAdjusted))
            || RobotUtils.isHubActiveForAlliance(
                alliance, autoWinner, getDisplayActiveHub(matchTimeRaw));

    // --- Primary countdown ---
    // Determine whether raw time still shows the hub as active for our alliance.
    // This is the source of truth for which time to count down from — we don't want the
    // countdown to snap to the TOF duration in the last effectiveTof seconds before inactive.
    boolean hubActiveByRaw =
        RobotUtils.isHubActiveForAlliance(alliance, autoWinner, getDisplayActiveHub(matchTimeRaw));

    if (currentPhase == MatchPhase.AUTO) {
      // Countdown to end of auto.
      primaryTimeLeft = matchTimeRaw;
    } else if (currentPhase == MatchPhase.ENDGAME) {
      if (matchTimeRaw <= RobotUtils.ENDGAME_START_SECONDS) {
        // Raw has also crossed — both alliances are in endgame. Count down to end of match.
        primaryTimeLeft = matchTimeRaw;
      } else {
        // TOF-early window: adjusted crossed ENDGAME_START but raw hasn't yet.
        // Keep counting with adjusted time so the display is smooth for the inactive alliance
        // (AUTOLOSER in shift 4) whose countdown was already running toward zero.
        // Active alliance (AUTOWINNER) uses raw time — no jump since raw is still in shift 4.
        double timeForCountdown = hubActiveByRaw ? matchTimeRaw : matchTimeAdjusted;
        primaryTimeLeft = getDisplayTimeUntilHubChange(timeForCountdown);
      }
    } else if (currentPhase == MatchPhase.TRANSITION) {
      // Both hubs active — no TOF offset. Show raw seconds until shift 1 begins.
      primaryTimeLeft = matchTimeRaw - RobotUtils.TRANSITION_END_SECONDS;
    } else {
      // Teleop shifts.
      // Hub active by raw → count down with raw time (no early cutoff, grace covers in-flight
      // balls).
      // Hub inactive by raw → count down with adjusted time (hits zero when a ball fired now
      //   would land while the hub is still active, i.e. the last safe moment to shoot).
      double timeForCountdown = hubActiveByRaw ? matchTimeRaw : matchTimeAdjusted;
      primaryTimeLeft = getDisplayTimeUntilHubChange(timeForCountdown);
    }
  }

  /**
   * Publishes all hub shift display values to SmartDashboard and logs them via AdvantageKit. Call
   * after {@link #update()} once per cycle.
   */
  public void log() {
    SmartDashboard.putString("Phase", currentPhase.name());
    SmartDashboard.putNumber("Teleop Shift", teleopShift + 1);
    SmartDashboard.putNumber("Time Left in Phase", primaryTimeLeft);
    SmartDashboard.putString("Active Hub", activeHub.toString());
    SmartDashboard.putBoolean("Can Score in Hub", hubActiveForDisplay);

    Logger.recordOutput("HubShift/Phase", currentPhase.name());
    Logger.recordOutput("HubShift/TeleopShift", teleopShift + 1);
    Logger.recordOutput("HubShift/TimeLeftInPhase", primaryTimeLeft);
    Logger.recordOutput("HubShift/ActiveHub", activeHub.toString());
    Logger.recordOutput("HubShift/HubActive", hubActive);
    Logger.recordOutput("HubShift/HubActiveDisplay", hubActiveForDisplay);
    Logger.recordOutput("HubShift/EffectiveTOF", getEffectiveTof());
  }

  /**
   * Returns whether our alliance's hub is currently active for shooting (shooter gate). Uses the
   * full grace-period logic — balls fired just before the shift boundary can still score.
   *
   * @return true if our hub accepts balls right now
   */
  public boolean isHubActive() {
    return hubActive;
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  /**
   * Returns the effective time-of-flight used for all shift boundary adjustments. Clamps to the
   * minimum mapped TOF to handle out-of-range poses, then adds the sensor delay.
   */
  private double getEffectiveTof() {
    return Math.max(cachedTimeOfFlight, hubShotCalculator.getMinTimeOfFlightSecs())
        + BALL_TO_SENSOR_DELAY_SECONDS;
  }

  private RobotUtils.ActiveHub getDisplayActiveHub(double adjustedMatchTime) {
    if (adjustedMatchTime > RobotUtils.TRANSITION_END_SECONDS) {
      return RobotUtils.ActiveHub.BOTH;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_1_END_SECONDS) {
      return RobotUtils.ActiveHub.AUTOLOSER;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_2_END_SECONDS) {
      return RobotUtils.ActiveHub.AUTOWINNER;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_3_END_SECONDS) {
      return RobotUtils.ActiveHub.AUTOLOSER;
    } else if (adjustedMatchTime > RobotUtils.ENDGAME_START_SECONDS) {
      return RobotUtils.ActiveHub.AUTOWINNER;
    } else {
      return RobotUtils.ActiveHub.BOTH;
    }
  }

  private double getDisplayTimeUntilHubChange(double adjustedMatchTime) {
    if (adjustedMatchTime > RobotUtils.TRANSITION_END_SECONDS) {
      return adjustedMatchTime - RobotUtils.TRANSITION_END_SECONDS;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_1_END_SECONDS) {
      return adjustedMatchTime - RobotUtils.SHIFT_1_END_SECONDS;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_2_END_SECONDS) {
      return adjustedMatchTime - RobotUtils.SHIFT_2_END_SECONDS;
    } else if (adjustedMatchTime > RobotUtils.SHIFT_3_END_SECONDS) {
      return adjustedMatchTime - RobotUtils.SHIFT_3_END_SECONDS;
    } else if (adjustedMatchTime > RobotUtils.ENDGAME_START_SECONDS) {
      return adjustedMatchTime - RobotUtils.ENDGAME_START_SECONDS;
    } else {
      return Math.max(0, adjustedMatchTime);
    }
  }
}
