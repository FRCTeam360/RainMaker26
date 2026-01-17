// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HALUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum RobotType {
    SIM,
    WOODBOT
  }

  public static class WoodBotConstants {
    public static final int INTAKE_ID = 0;
    public static final int INTAKE_PORT = 0;
    public static final int INTAKE_SENSOR_PORT = 0;
    public static final int INTAKE_PIVOT_PORT = 0;

    public static final int INDEXER_SENSOR_PORT = 0;
    public static final int INDEXER_ID = 1;

    public static final int FLYWHEEL0_ID = 0;
    public static final int FLYWHEEL1_ID = 0;

    public static final int HOOD_ID = 0;

    public static final String CANBUS_NAME = "";
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SerialAddressConstants {
    public static String WOOD_SERIAL_ADDRESS = "";
  }

  public static RobotType getRobotType() {
    String serialAddress = HALUtil.getSerialNumber();

    if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
      return Constants.RobotType.WOODBOT;
    }
    // else if (!Robot.isReal()) { // KEEP AT BOTTOM
    // return Constants.RobotType.SIM;
    // }
    return Constants.RobotType.SIM;
  }
}
