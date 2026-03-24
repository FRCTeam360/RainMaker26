package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.io.File;
import java.util.Optional;

public class RobotUtils {
  private static final double SHIFT_GRACE_PERIOD_SECONDS = 2.0;

  public static final double TRANSITION_END_SECONDS = 130;
  public static final double SHIFT_1_END_SECONDS = 105;
  public static final double SHIFT_2_END_SECONDS = 80;
  public static final double SHIFT_3_END_SECONDS = 55;
  public static final double ENDGAME_START_SECONDS = 30;

  public enum ActiveHub {
    BOTH,
    AUTOLOSER,
    AUTOWINNER,
  }

  public static boolean isUsbWriteable() {
    File usb = new File(Constants.IOConstants.USB_ROOT_DIRECTORY);
    if (usb.exists() && usb.isDirectory()) {
      try {

        File temporaryFile = File.createTempFile("usb", ".txt", usb);
        temporaryFile.delete();
        return true;
      } catch (Exception e) {
        System.out.println("usb not found");
      }
    }
    return false;
  }

  /**
   * Returns the alliance that won auto based on the gameSpecificMessage from DriverStation by
   * checking the first character of the gameSpecificMessage
   *
   * @param autoWinner the gameSpecificMessage from DriverStation
   * @return the alliance that won the autonomous period
   */
  public static Alliance getAutoWinner(String autoWinner) {
    // the game specific message tells you which alliance won auto
    if (autoWinner.length() > 0) {
      // checks which hub is open
      switch (autoWinner.charAt(0)) {
        case 'B':
          return Alliance.Blue;
        case 'R':
          return Alliance.Red;
        default:
          return null;
      }
    }
    // called when no data was received from driver station
    return null;
  }

  /**
   * Returns which alliance's hub is active based on the gameTime from DriverStation
   *
   * @param gameTime the gameTime from DriverStation
   * @param isTele if the game is in teleop or auto. Can be accessed by DriverStation.isTeleop()
   * @param timeOfFlight the time of flight of the shot in seconds
   * @return which hub(s) are currently active
   */
  public static ActiveHub getActiveHub(double gameTime, Boolean isTele, double timeOfFlight) {
    // gameTime is the getMatchTime() from DriverStation, isTele is the isTeleop() from
    // DriverStation
    ActiveHub activeHub = ActiveHub.BOTH;
    gameTime -= timeOfFlight;
    // Sets phases based on the current time in the game
    if (!isTele) {
      activeHub = ActiveHub.BOTH; // AUTO
    } else if (isTele) {
      if (gameTime <= ENDGAME_START_SECONDS) {
        activeHub = ActiveHub.BOTH; // END GAME
      } else if (gameTime < SHIFT_3_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 4
      } else if (gameTime <= SHIFT_3_END_SECONDS
          && gameTime >= SHIFT_3_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (gameTime < SHIFT_2_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOLOSER; // ALLIANCE SHIFT 3
      } else if (gameTime <= SHIFT_2_END_SECONDS
          && gameTime >= SHIFT_2_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (gameTime < SHIFT_1_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 2
      } else if (gameTime <= SHIFT_1_END_SECONDS
          && gameTime >= SHIFT_1_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (gameTime <= TRANSITION_END_SECONDS) {
        activeHub = ActiveHub.AUTOLOSER; // ALLIANCE SHIFT 1
      } else {
        return ActiveHub.BOTH; // TRANSITION
      }
    }
    return activeHub;
  }

  /**
   * Returns the active hub for display purposes, with grace periods invisible to the driver.
   *
   * <p>The grace period (BOTH) at each shift boundary is for the opponent's in-flight balls — from
   * our alliance's perspective, our hub goes active immediately at the raw boundary with no delay,
   * and goes inactive immediately at the next raw boundary with no grace. So the display simply
   * uses the raw {@code SHIFT_X_END_SECONDS} boundaries, and grace is silently absorbed into the
   * adjacent active window.
   *
   * <p>Boundaries use the raw shift times so the display flips at the same moment as {@link
   * #getDisplayTimeUntilHubChange}. Pass the TOF-adjusted match time so the display turns active
   * exactly when a ball fired now would land while the hub is still active.
   *
   * <p>Phase map (adjustedMatchTime = raw - TOF - sensorDelay):
   *
   * <pre>
   *  > 130         BOTH        (transition — both always active)
   *  105–130       AUTOLOSER   (shift 1: autoloser active, autowinner inactive)
   *   80–105       AUTOWINNER  (shift 2: autowinner active, autoloser inactive; includes 103–105 grace)
   *   55–80        AUTOLOSER   (shift 3: autoloser active, autowinner inactive; includes 78–80 grace)
   *   30–55        AUTOWINNER  (shift 4: autowinner active, autoloser inactive; includes 53–55 grace)
   *  ≤ 30          BOTH        (endgame — both always active)
   * </pre>
   *
   * @param adjustedMatchTime match time remaining minus effective time of flight, in seconds
   * @return which hub(s) are active for display purposes
   */
  public static ActiveHub getDisplayActiveHub(double adjustedMatchTime) {
    if (adjustedMatchTime > TRANSITION_END_SECONDS) {
      return ActiveHub.BOTH; // transition
    } else if (adjustedMatchTime > SHIFT_1_END_SECONDS) {
      return ActiveHub.AUTOLOSER; // shift 1 (inactive for autowinner, active for autoloser)
    } else if (adjustedMatchTime > SHIFT_2_END_SECONDS) {
      return ActiveHub.AUTOWINNER; // shift 2 + grace folded in
    } else if (adjustedMatchTime > SHIFT_3_END_SECONDS) {
      return ActiveHub.AUTOLOSER; // shift 3 + grace folded in
    } else if (adjustedMatchTime > ENDGAME_START_SECONDS) {
      return ActiveHub.AUTOWINNER; // shift 4 + grace folded in
    } else {
      return ActiveHub.BOTH; // endgame
    }
  }

  /**
   * Returns the display countdown until the next hub phase change. Grace periods are folded into
   * the adjacent active window, so the countdown and {@link #getDisplayActiveHub} always flip at
   * the same boundary. Uses TOF-adjusted time so the countdown hits zero exactly when a ball fired
   * now would land at the phase boundary.
   *
   * @param adjustedMatchTime match time remaining minus effective time of flight, in seconds
   * @return seconds until the next display hub phase change, or 0 if in endgame/done
   */
  public static double getDisplayTimeUntilHubChange(double adjustedMatchTime) {
    if (adjustedMatchTime > TRANSITION_END_SECONDS) {
      return adjustedMatchTime - TRANSITION_END_SECONDS;
    } else if (adjustedMatchTime > SHIFT_1_END_SECONDS) {
      return adjustedMatchTime - SHIFT_1_END_SECONDS;
    } else if (adjustedMatchTime > SHIFT_2_END_SECONDS) {
      return adjustedMatchTime - SHIFT_2_END_SECONDS;
    } else if (adjustedMatchTime > SHIFT_3_END_SECONDS) {
      return adjustedMatchTime - SHIFT_3_END_SECONDS;
    } else if (adjustedMatchTime > ENDGAME_START_SECONDS) {
      return adjustedMatchTime - ENDGAME_START_SECONDS;
    } else {
      return Math.max(0, adjustedMatchTime);
    }
  }

  /**
   * Returns if our alliance's hub is active based on which alliance we're on, which alliance won
   * auto, and which hub(s) are open
   *
   * @param alliance the alliance we're on from DriverStation
   * @param autoWinner the alliance that won auto
   * @param gamePhase which hub(s) are active (auto winner's or auto loser's)
   * @return if our alliance's hub is active
   */
  public static boolean isHubActiveForAlliance(
      Optional<Alliance> alliance, Alliance autoWinner, ActiveHub gamePhase) {
    // alliance is our alliance, autoWinner is the result of getAutoWinner, gamePhase is the result
    // of getActiveHub
    boolean hubActive = true;
    if (alliance.isPresent()) {
      if (gamePhase == null || autoWinner == null) {
        return true;
      }
      switch (gamePhase) {
          // during auto, transitional phase, and end game
        case BOTH:
          return true;
          // during alliance shifts 1 and 3:
        case AUTOLOSER:
          if (alliance.get() == autoWinner) {
            hubActive = false;
          } else {
            hubActive = true;
          }
          return hubActive;
          // during alliance shifts 2 and 4:
        case AUTOWINNER:
          if (alliance.get() == autoWinner) {
            hubActive = true;
          } else {
            hubActive = false;
          }
          return hubActive;
        default:
      }
    } else {
      // this is called when no alliance has been received from driver station
      return true;
    }
    return hubActive;
  }
}
