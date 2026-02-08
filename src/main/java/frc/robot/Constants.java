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
    WOODBOT,
    REPLAY
  }

  public static class WoodBotConstants {
    // === INTAKE ===
    public static final int INTAKE_SENSOR_PORT = 0;
    public static final int INTAKE_ID = 15;
    public static final int INTAKE_PIVOT_ID = 0;

    // === HOPPER ===
    public static final int INDEXER_SENSOR_PORT = 1;
    public static final int INDEXER_SENSOR_ID = 2;
    public static final int INDEXER_ID = 16;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_KICKER_ID = 17;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 3;
    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    // === HOOD ===
    public static final int HOOD_ID = 20;

    // === LIMELIGHT ===
    public static final String LIMELIGHT = "limelight";

    // === CANBUS ===
    public static final String CANBUS_NAME = "Default Name";
  }

  public static class SimulationConstants {
    // === INTAKE ===
    public static final int INTAKE_MOTOR = 12;
    public static final int INTAKE_SENSOR_PORT = 10;
    public static final int INTAKE_PIVOT_MOTOR = 15;

    // === HOPPER ===
    public static final int INDEXER_MOTOR = 9;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_KICKER_MOTOR = 18;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 19;
    public static final int FLYWHEEL_MOTOR = 2;

    // === HOOD ===
    public static final int HOOD_MOTOR = 6;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SerialAddressConstants {
    public static String WOOD_SERIAL_ADDRESS = "032BE44A";
  }

  public static RobotType getRobotType() {
    String serialAddress = HALUtil.getSerialNumber();

    if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
      return Constants.RobotType.WOODBOT;
    } else if (!Robot.isReal()) { // KEEP AT BOTTOM
      return Constants.RobotType.SIM;
    }
    return Constants.RobotType.WOODBOT;
  }
}
