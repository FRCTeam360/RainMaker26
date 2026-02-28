// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static enum RobotType {
    SIM,
    WOODBOT,
    PRACTICEBOT,
    REPLAY
  }

  public static final class IOConstants {
    // === USB PATHS ===
    public static final String USB_ROOT_DIRECTORY = "/U";
  }

  public static final CANBus RIO_CANBUS = new CANBus("rio");

  public static class WoodBotConstants {
    // === INTAKE ===
    public static final int INTAKE_SENSOR_PORT = 0;
    public static final int INTAKE_ID = 15;
    public static final int INTAKE_PIVOT_ID = 0;

    // === HOPPER ===
    public static final int INDEXER_SENSOR_PORT = 1;
    public static final int INDEXER_ID = 16;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_KICKER_ID = 17;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 2;
    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    // === HOOD ===
    public static final int HOOD_ID = 20;

    // === LIMELIGHT ===
    public static final String LIMELIGHT_3 = "limelight";
    public static final String LIMELIGHT_4 = "limelight-two";

    // === CANBUS ===
    public static final CANBus CANBUS = new CANBus("Default Name");

    // === SHOT CALCULATOR ===
    public static final InterpolatingDoubleTreeMap shotHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    public static final double MIN_SHOT_DISTANCE_METERS = 0.0;
    public static final double MAX_SHOT_DISTANCE_METERS = 5.0;

    static {
      // === SHOOTING VALUES ===
      shotHoodAngleMap.put(5.0, 20.0);
      shotHoodAngleMap.put(4.0, 18.0);
      shotHoodAngleMap.put(3.0, 16.0);
      shotHoodAngleMap.put(2.0, 11.0); // THIS IS GOOD
      shotHoodAngleMap.put(1.0, 8.0); // THIS IS GOOD
      shotHoodAngleMap.put(0.0, 6.0);

      // === SHOOTING VALUES ===
      shotFlywheelSpeedMap.put(5.0, 3750.0);
      shotFlywheelSpeedMap.put(4.0, 3750.0);
      shotFlywheelSpeedMap.put(3.0, 3375.0);
      shotFlywheelSpeedMap.put(2.0, 3000.0); // THIS IS GOOD
      shotFlywheelSpeedMap.put(0.0, 2750.0);

      timeOfFlightMap.put(0.0, 0.0);

      // === PASSING VALUES === (TODO: change placeholder values)
      passFlywheelSpeedMap.put(6.0, 4000.0);

      // === PASSING VALUES === (TODO: change placeholder values)
      passHoodAngleMap.put(6.0, 22.0);
    }
  }

  public static class PracticeBotConstants {
    // === SHOT CALCULATOR ===
    public static final InterpolatingDoubleTreeMap shotHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    public static final double MIN_SHOT_DISTANCE_METERS = 0.0;
    public static final double MAX_SHOT_DISTANCE_METERS = 6.0;

    // === INTAKE ===
    public static final int INTAKE_PIVOT_ID = 14;
    public static final int INTAKE_ID = 15;

    // === CLIMBER ===
    public static final int CLIMBER_RIGHT_ID = 26;
    public static final int CLIMBER_LEFT_ID = 27;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    // === FLYWHEEL KICKER ===
    public static final int FLYWHEEL_KICKER_ID = 20;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 21;

    // === HOPPER ===
    public static final int HOPPER_ROLLER_ID = 22;
    public static final int TWINDEXER_ID = 23;
    // public static final int HOPPER_SENSOR_ID = 25;

    // === HOOD ===
    public static final int HOOD_ID = 24;

    // === LIMELIGHT ===
    public static final String LIMELIGHT = "limelight-right";

    // === CANBUS ===
    public static final CANBus CANBUS = new CANBus("Default Name");

    static {
      shotHoodAngleMap.put(6.0, 18.0);
      shotHoodAngleMap.put(5.0, 18.0); // TESTED
      shotHoodAngleMap.put(4.0, 15.0);
      shotHoodAngleMap.put(3.0, 6.0); // TESTED
      shotHoodAngleMap.put(2.5, 4.0); // TESTED
      shotHoodAngleMap.put(2.0, 2.0);
      shotHoodAngleMap.put(1.0, 0.0);
      shotHoodAngleMap.put(0.0, 0.0);

      // === SHOOTING VALUES ===
      shotFlywheelSpeedMap.put(6.0, 2500.0);
      shotFlywheelSpeedMap.put(5.0, 2500.0); // TESTED
      shotFlywheelSpeedMap.put(4.0, 2250.0);
      shotFlywheelSpeedMap.put(3.0, 2250.0); // TESTED
      shotFlywheelSpeedMap.put(2.5, 2150.0); // TESTED
      shotFlywheelSpeedMap.put(2.0, 2000.0);
      shotFlywheelSpeedMap.put(1.0, 1800.0);
      shotFlywheelSpeedMap.put(0.0, 2000.0);

      timeOfFlightMap.put(0.0, 0.0);
    }
  }

  public static class SimulationConstants {
    public static final double SIM_TICK_RATE_S = 0.02;

    // === INTAKE ===
    public static final int INTAKE_MOTOR = 30;
    public static final int INTAKE_SENSOR_PORT = 10;
    public static final int INTAKE_PIVOT_MOTOR = 15;

    // === HOPPER ===
    public static final int INDEXER_MOTOR = 31;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_KICKER_MOTOR = 18;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 19;
    public static final int FLYWHEEL_MOTOR = 32;

    // === HOPPER ROLLER ===
    public static final int HOPPER_ROLLER_MOTOR = 35;

    // === HOOD ===
    public static final int HOOD_MOTOR = 34;

    // === CLIMBER ===
    public static final int CLIMBER_MOTOR = 36;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SerialAddressConstants {
    public static final String WOOD_SERIAL_ADDRESS = "032BE44A";
    public static final String PRACTICE_SERIAL_ADDRESS = "03260AD5";
  }

  public static double loopPeriodSecs; // add value

  public static RobotType getRobotType() {
    String serialAddress = HALUtil.getSerialNumber();

    if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
      return Constants.RobotType.WOODBOT;
    } else if (serialAddress.equals(SerialAddressConstants.PRACTICE_SERIAL_ADDRESS)) {
      return Constants.RobotType.PRACTICEBOT;
    } else if (!Robot.isReal()) { // KEEP AT BOTTOM
      return Constants.RobotType.SIM;
    }
    return Constants.RobotType.PRACTICEBOT;
  }
}
