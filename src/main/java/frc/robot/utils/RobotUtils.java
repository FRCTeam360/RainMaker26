package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.File;

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
    return false;
  }
}
