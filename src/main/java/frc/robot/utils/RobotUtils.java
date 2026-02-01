package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.File;
import java.util.Optional;

public class RobotUtils {
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

  public static Alliance hubData() {
    // the game specific message doesn't tell you which hub is active, it tells you which hub is
    // active for phases 2 and 4
    String hubData = DriverStation.getGameSpecificMessage();
    if (hubData.length() > 0) {
      // checks which hub is open
      switch (hubData.charAt(0)) {
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

  public static String getHubPhase() {
    double gameTime = DriverStation.getMatchTime();
    String hubPhase = null;
    // Sets phases based on the current time in the game
    if (DriverStation.isAutonomous()) {
      hubPhase = "Both";
    } else if (DriverStation.isTeleop()) {
      if (gameTime <= 140) hubPhase = "Both";
      if (gameTime <= 130) hubPhase = "Shift odd";
      if (gameTime <= 105) hubPhase = "Shift even";
      if (gameTime <= 80) hubPhase = "Shift odd";
      if (gameTime <= 55) hubPhase = "Shift even";
      if (gameTime <= 30) hubPhase = "Both";
    }
    return hubPhase;
  }

  public static Boolean hubActive() {
    Boolean hubActive = null;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Alliance hubData = hubData();
    String gamePhase = getHubPhase();

    if (alliance.isPresent()) {
      switch (gamePhase) {
          // during auto, transitional phase, and end game
        case "Both":
          hubActive = true;
          return hubActive;
          // during alliance shifts 1 and 3
        case "Phase odd":
          // if we're a part of the blue alliance
          if (alliance.get() == Alliance.Blue) {
            if (hubData == Alliance.Blue) hubActive = false;
            if (hubData == Alliance.Red) hubActive = true;
          }
          // if we're a part of the red alliance
          if (alliance.get() == Alliance.Red) {
            if (hubData == Alliance.Blue) hubActive = true;
            if (hubData == Alliance.Red) hubActive = false;
          }
          return hubActive;
          // during alliance shifts 2 and 4
        case "Phase even":
          if (alliance.get() == Alliance.Blue) {
            if (hubData == Alliance.Blue) hubActive = true;
            if (hubData == Alliance.Red) hubActive = false;
          }
          if (alliance.get() == Alliance.Red) {
            if (hubData == Alliance.Blue) hubActive = false;
            if (hubData == Alliance.Red) hubActive = true;
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
