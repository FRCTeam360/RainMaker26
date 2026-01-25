// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static enum RobotType {
    SIM,
    WOODBOT,
    REPLAY
  }

  public static class WoodBotConstants {
    public static final int INTAKE_ID = 15;
    public static final int INTAKE_SENSOR_PORT = 0;

    public static final int INDEXER_SENSOR_PORT = 1;
    public static final int INTAKE_PIVOT_ID = 0;
    public static final int INDEXER_SENSOR_ID = 2;
    public static final int INDEXER_ID = 16;
    public static final int FLYWHEEL_KICKER_ID = 17;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 3;

    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    public static final int HOOD_ID = 0;

    public static final String CANBUS_NAME = "Default Name";
  }

  public static class SimulationConstants {
    public static final int FLYWHEEL_ENCODER1 = 0;
    public static final int FLYWHEEL_ENCODER2 = 1;
    public static final int FLYWHEEL_MOTOR = 2;

    public static final int HOOD_ENCODER1 = 3;
    public static final int HOOD_ENCODER2 = 4;
    public static final int HOOD_MOTOR = 5;

    public static final int INDEXER_ENCODER1 = 6;
    public static final int INDEXER_ENCODER2 = 7;
    public static final int INDEXER_MOTOR = 8;

    public static final int INTAKE_ENCODER1 = 9;
    public static final int INTAKE_ENCODER2 = 10;
    public static final int INTAKE_MOTOR = 11;

    public static final int INTAKE_PIVOT_ENCODER1 = 12;
    public static final int INTAKE_PIVOT_ENCODER2 = 13;
    public static final int INTAKE_PIVOT_MOTOR = 14;
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
    }
    // else if (!Robot.isReal()) { // KEEP AT BOTTOM
    // return Constants.RobotType.SIM;
    // }
    return Constants.RobotType.SIM;
  }
}
