package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.io.File;

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
          return null;
      }
    }
    // called when no data was received from driver station
    return null;
  }
}
