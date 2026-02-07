package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.File;
import java.util.Optional;

public class RobotUtils {
  enum ActiveHub {
    BOTH,
    AUTOLOSER,
    AUTOWINNER,
  }

  public static boolean isUsbWriteable() {
    File usb = new File("/U");
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

  public static Alliance getAutoWinner() {
    // the game specific message doesn't tell you which hub is active, it tells you which hub is
    // active for phases 2 and 4
    String autoWinner = DriverStation.getGameSpecificMessage();
    if (autoWinner.length() > 0) {
      // checks which hub is open
      switch (autoWinner.charAt(0)) {
        case 'B':
          return Alliance.Blue;
        case 'R':
          return Alliance.Red;
        default:
          // only called when there's an invalid character for the game specific message
          break;
      }
    }
    // called when no data was received from driver station
    return null;
  }

  public static ActiveHub getHubPhase(double gameTime) {
    // double gameTime = DriverStation.getMatchTime();
    ActiveHub activeHub = null;
    // Sets phases based on the current time in the game
    if (DriverStation.isAutonomous()) {
      activeHub = ActiveHub.BOTH; // AUTO
    } else if (DriverStation.isTeleop()) {
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
      } else if (gameTime <= 140) {
        activeHub = ActiveHub.BOTH; // TRANSITION
      }
    }
    return activeHub;
  }

  public static Boolean hubActive() {
    Boolean hubActive = null;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Alliance autoWinner = getAutoWinner();
    ActiveHub gamePhase = getHubPhase(DriverStation.getMatchTime());

    if (alliance.isPresent()) {
      switch (gamePhase) {
          // during auto, transitional phase, and end game
        case BOTH:
          hubActive = true;
          return hubActive;
          // during alliance shifts 1 and 3:
        case AUTOLOSER:
          // if we're a part of the blue alliance
          if (alliance.get() == Alliance.Blue) {
            if (autoWinner == Alliance.Blue) hubActive = false;
            if (autoWinner == Alliance.Red) hubActive = true;
          } else if (alliance.get() == Alliance.Red) {
            // if we're a part of the red alliance
            if (autoWinner == Alliance.Blue) hubActive = true;
            if (autoWinner == Alliance.Red) hubActive = false;
          }
          return hubActive;
          // during alliance shifts 2 and 4:
        case AUTOWINNER:
          if (alliance.get() == Alliance.Blue) {
            if (autoWinner == Alliance.Blue) hubActive = true;
            if (autoWinner == Alliance.Red) hubActive = false;
          } else if (alliance.get() == Alliance.Red) {
            if (autoWinner == Alliance.Blue) hubActive = false;
            if (autoWinner == Alliance.Red) hubActive = true;
          }
          return hubActive;
        default:
          break;
      }
    } else {
      // this is called when no alliance has been received from driver station
      hubActive = null;
    }
    return hubActive;
  }
}
