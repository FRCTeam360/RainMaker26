package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.File;
import java.util.Optional;
import frc.robot.Constants;

public class RobotUtils {
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
   * @return which hub(s) are currently active
   */
  public static ActiveHub getHubPhase(double gameTime, Boolean isTele) {
    // gameTime is the getMatchTime() from DriverStation, isTele is the isTeleop() from
    // DriverStation
    ActiveHub activeHub = null;
    // Sets phases based on the current time in the game
    if (isTele == false) {
      activeHub = ActiveHub.BOTH; // AUTO
    } else if (isTele == true) {
      if (gameTime <= 30) {
        activeHub = ActiveHub.BOTH; // END GAME
      } else if (gameTime <= 55) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 4
      } else if (gameTime <= 80) {
        activeHub = ActiveHub.AUTOLOSER; // ALLIANCE SHIFT 3
      } else if (gameTime <= 105) {
        activeHub = ActiveHub.AUTOWINNER; // ALLIANCE SHIFT 2
      } else if (gameTime <= 130) {
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
  public static Boolean hubActive(
      Optional<Alliance> alliance, Alliance autoWinner, ActiveHub gamePhase) {
    // alliance is our alliance, autoWinner is the result of getAutoWinner, gamePhase is the result
    // of getHubPhase
    Boolean hubActive = null;
    if (alliance.isPresent()) {
      if (gamePhase == null) {
        return null;
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
      return null;
    }
    return hubActive;
  }
}
