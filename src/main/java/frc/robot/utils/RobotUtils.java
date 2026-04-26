package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.io.File;
import java.util.Optional;

public class RobotUtils {
  private static final double SHIFT_GRACE_PERIOD_SECONDS = 2.0;
  private static final double HUB_TO_SENSOR_SECONDS = 1.0;

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
   * Returns whether our alliance won the autonomous period.
   *
   * @return true if our alliance is the auto winner, false otherwise (including if alliance or game
   *     message is unavailable)
   */
  public static boolean isAutoWinner() {
    Alliance autoWinner = getAutoWinner(DriverStation.getGameSpecificMessage());
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && autoWinner == alliance.get();
  }

  public static boolean hasAutoCheckpointElapsed(double checkpoint) {
    if (DriverStation.isAutonomousEnabled()) {
      double currentTime = DriverStation.getMatchTime();
      if (currentTime <= checkpoint) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  /**
   * Returns which alliance's hub is active based on the gameTime from DriverStation
   *
   * @param gameTime the gameTime from DriverStation
   * @param isTele if the game is in teleop or auto. Can be accessed by DriverStation.isTeleop()
   * @param timeOfFlight the time of flight of the shot in seconds
   * @return which hub(s) are currently active
   */
  public static ActiveHub getActiveHubAtShotLanding(
      double gameTime, Boolean isTele, double timeOfFlight, double indexerToFlywheelSeconds) {
    // gameTime is the getMatchTime() from DriverStation, isTele is the isTeleop() from
    // DriverStation
    ActiveHub activeHub = ActiveHub.BOTH;
    double timeAtShotLanding = gameTime - (timeOfFlight + indexerToFlywheelSeconds);
    // Sets phases based on the current time in the game
    if (!isTele) {
      activeHub = ActiveHub.BOTH; // AUTO
    } else if (isTele) {
      if (timeAtShotLanding <= ENDGAME_START_SECONDS) {
        activeHub = ActiveHub.BOTH; // END GAME
      } else if (timeAtShotLanding < SHIFT_3_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 4
      } else if (timeAtShotLanding <= SHIFT_3_END_SECONDS
          && timeAtShotLanding >= SHIFT_3_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (timeAtShotLanding < SHIFT_2_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOLOSER; // ALLIANCE SHIFT 3
      } else if (timeAtShotLanding <= SHIFT_2_END_SECONDS
          && timeAtShotLanding >= SHIFT_2_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (timeAtShotLanding < SHIFT_1_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 2
      } else if (timeAtShotLanding <= SHIFT_1_END_SECONDS
          && timeAtShotLanding >= SHIFT_1_END_SECONDS - SHIFT_GRACE_PERIOD_SECONDS) {
        activeHub = ActiveHub.BOTH; // ALLIANCE SHIFT GRACE PERIOD
      } else if (timeAtShotLanding <= TRANSITION_END_SECONDS) {
        activeHub = ActiveHub.AUTOLOSER; // ALLIANCE SHIFT 1
      } else {
        return ActiveHub.BOTH; // TRANSITION
      }
    }
    return activeHub;
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
