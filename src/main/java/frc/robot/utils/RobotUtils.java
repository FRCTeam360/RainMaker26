package frc.robot.utils;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    String hubData = DriverStation.getGameSpecificMessage();
    if(hubData.length() > 0){
      char allianceChar = hubData.charAt(0);
      //checks which hub is open
      switch(hubData.charAt(0)){
        case 'B':
        return Alliance.Blue;
        case 'R':
        return Alliance.Red;
        default:
        //only called when there's an invalid character for the game specific message
        throw new IllegalStateException("Invalid character for alliance configuration from game specific message: " + allianceChar);
      }
    } 
    //called when no data was received from driver station
    throw new IllegalStateException("Driver station could not receive game specific message");
  }
}