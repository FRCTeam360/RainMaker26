package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.io.File;

public class RobotUtils {
  public static final double SHIFT_TIME_SECONDS = 25;
  public static final double TIME_TO_SCORE = 2.0; // TODO set to real value
  public static final double TRANSITION_END_SECONDS_SHOOTING = 130 + TIME_TO_SCORE;
  public static final double SHIFT_1_END_SECONDS_SHOOTING = 105 + TIME_TO_SCORE;
  public static final double SHIFT_2_END_SECONDS_SHOOTING = 80 + TIME_TO_SCORE;
  public static final double SHIFT_3_END_SECONDS_SHOOTING = 55 + TIME_TO_SCORE;
  public static final double ENDGAME_START_SECONDS_SHOOTING = 30 + TIME_TO_SCORE;


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
