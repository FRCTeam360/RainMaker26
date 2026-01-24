package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
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

  public static Alliance getAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue won AUTO - Blue hub inactive in Shift 1 & 3
          return Alliance.Blue;
        case 'R':
          // Red won AUTO - Red hub inactive in Shift 1 & 3
          return Alliance.Red;
        default:
          // Corrupt data
      }
    }
    return null;
  }

  public static boolean canShoot(double estTOF) {
    // Can always shoot in auto
    if (DriverStation.isAutonomous()) return true;

    // Can always shoot when not connected to FMS and not in a test match
    if (!DriverStation.isFMSAttached() && DriverStation.getMatchType() == MatchType.None) {
      return true;
    }

    Alliance autoWinner = getAutoWinner();
    Optional<Alliance> myAlliance = DriverStation.getAlliance();

    // If we can't determine alliances, allow shooting
    if (autoWinner == null || myAlliance.isEmpty()) {
      return true;
    }

    // Driver station time on the field is in increments of one second
    // TODO: find a more precise way to track match time, possibly on autoinit
    double timeAtScore = DriverStation.getMatchTime() - estTOF;

    if(timeAtScore > 130.0 || timeAtScore <= 30.0) return true;

    // Determine which shift the ball will arrive in
    double shift = Math.floor((timeAtScore - 30.0) / 25.0);

    // If my alliance won auto, my hub is inactive in Shifts 1 & 3
    // If opposing alliance won auto, my hub is inactive in Shift 2
    boolean myAllianceWonAuto = autoWinner == myAlliance.get();

    if (myAllianceWonAuto) {
      // Can only shoot in even shifts
      return shift % 2 == 0;
    }
    // Can shoot in odd shifts
    return shift % 2 != 0;
  }
}
