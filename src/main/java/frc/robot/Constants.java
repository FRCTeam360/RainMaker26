// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // This is to load the apriltag field layout on robot initialization.
  // It prevents our robot code from having a 5 second initial lag on enablement
  // after new code is
  // deployed.
  // This is load bearing code like that coconut jpg that keeps TF2 running -_-
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static Alliance AUTO_WINNER;

  public static enum RobotType {
    SIM,
    WOODBOT,
    COMPBOT,
    PRACTICEBOT,
    REPLAY
  }

  /** Frames to skip between processed frames while disabled. Only affects Limelight 4. */
  public static final int DISABLED_THROTTLE_SKIP_FRAMES = 200;

  /** Frames to skip between processed frames while enabled. Only affects Limelight 4. */
  public static final int ENABLED_THROTTLE_SKIP_FRAMES = 0;

  static RobotType robotType;

  public static LinearVelocity getMaxSpeed() {
    switch (getRobotType()) {
      case WOODBOT:
        return WoodBotConstants.maxSpeed;
      case COMPBOT:
        return CompBotConstants.maxSpeed;
      case PRACTICEBOT:
        return PracticeBotConstants.maxSpeed;
      case SIM:
        return SimulationConstants.maxSpeed;
      default:
        return PracticeBotConstants.maxSpeed;
    }
  }

  public static AngularVelocity getMaxAngularVelocity() {
    switch (getRobotType()) {
      case WOODBOT:
        return WoodBotConstants.maxAngularVelocity;
      case COMPBOT:
        return CompBotConstants.maxAngularVelocity;
      case PRACTICEBOT:
        return PracticeBotConstants.maxAngularVelocity;
      case SIM:
        return SimulationConstants.maxAngularVelocity;
      default:
        return PracticeBotConstants.maxAngularVelocity;
    }
  }

  public static RobotType getRobotType() {
    if (robotType != null) {
      return robotType;
    }
    return initRobotType();
  }

  private static RobotType initRobotType() {
    if (!Robot.isReal()) { // Check sim first to avoid empty serial address matching COMP
      robotType = Constants.RobotType.SIM;
      return robotType;
    }

    String serialAddress = HALUtil.getSerialNumber();

    if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
      robotType = Constants.RobotType.WOODBOT;
    } else if (!SerialAddressConstants.COMP_SERIAL_ADDRESS.isEmpty()
        && serialAddress.equals(SerialAddressConstants.COMP_SERIAL_ADDRESS)) {
      robotType = Constants.RobotType.COMPBOT;
    } else if (serialAddress.equals(SerialAddressConstants.PRACTICE_SERIAL_ADDRESS)) {
      robotType = Constants.RobotType.PRACTICEBOT;
    } else {
      robotType = Constants.RobotType.COMPBOT;
    }
    return robotType;
  }

  public static final class IOConstants {
    // === USB PATHS ===
    public static final String USB_ROOT_DIRECTORY = "/U";

    // === POWER DISTRIBUTION ===
    public static final int PDH_CAN_ID = 1; // REV PDH default CAN ID
  }

  public static final CANBus RIO_CANBUS = new CANBus("rio");

  public static class WoodBotConstants {
    // === INTAKE ===
    public static final int INTAKE_ROLLER_SENSOR_PORT = 0;
    public static final int INTAKE_ROLLER_ID = 15;
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

    // === MAXIMUMS ===
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.85);
    public static final AngularVelocity maxAngularVelocity = RevolutionsPerSecond.of(4.0);

    // === CANBUS ===
    public static final CANBus CANBUS = new CANBus("Default Name");

    // === PATHPLANNER CONFIG (as of Feb 20th tuning) ===
    public static final double MASS_KG = 60.0;
    public static final double MOI = 4.5;
    public static final double WHEEL_RADIUS_METERS = 0.048;
    public static final double MAX_DRIVE_SPEED_MPS = 4.69;
    public static final double WHEEL_COF = 1.3;
    public static final double DRIVE_GEARING = 6.03;
    public static final double DRIVE_CURRENT_LIMIT_AMPS = 80.0;
    public static final double MODULE_OFFSET_METERS = 0.301;

    /**
     * Creates a hardcoded RobotConfig for the WoodBot using known tuned constants.
     *
     * @return RobotConfig for the WoodBot
     */
    public static RobotConfig createPathPlannerConfig() {
      ModuleConfig moduleConfig =
          new ModuleConfig(
              WHEEL_RADIUS_METERS,
              MAX_DRIVE_SPEED_MPS,
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1),
              DRIVE_GEARING,
              DRIVE_CURRENT_LIMIT_AMPS,
              1);

      return new RobotConfig(
          MASS_KG,
          MOI,
          moduleConfig,
          new Translation2d(MODULE_OFFSET_METERS, MODULE_OFFSET_METERS),
          new Translation2d(MODULE_OFFSET_METERS, -MODULE_OFFSET_METERS),
          new Translation2d(-MODULE_OFFSET_METERS, MODULE_OFFSET_METERS),
          new Translation2d(-MODULE_OFFSET_METERS, -MODULE_OFFSET_METERS));
    }

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

    public static final double INDEXER_TO_FLYWHEEL_SECONDS = 0.4;

    public static final double MIN_SHOT_DISTANCE_METERS = 1.0;
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
      shotFlywheelSpeedMap.put(5.0, 3250.0);
      shotFlywheelSpeedMap.put(4.0, 3250.0);
      shotFlywheelSpeedMap.put(3.0, 3000.0);
      shotFlywheelSpeedMap.put(2.0, 2500.0); // THIS IS GOOD
      shotFlywheelSpeedMap.put(0.0, 2250.0);

      timeOfFlightMap.put(1.939, 0.82);
      timeOfFlightMap.put(3.011, 1.26);
      timeOfFlightMap.put(4.704, 1.37);

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

    public static final double HOOD_OFFSET = 2.0;

    public static final double INDEXER_TO_FLYWHEEL_SECONDS = 0.4;

    public static final double MIN_SHOT_DISTANCE_METERS = 1.25;
    public static final double MAX_SHOT_DISTANCE_METERS = 6.0;

    public static final double MIN_PASS_DISTANCE_METERS = 1.0;
    public static final double MAX_PASS_DISTANCE_METERS = 12.0;

    // === INTAKE ===
    public static final int INTAKE_PIVOT_ID = 14;
    public static final int LEFT_INTAKE_ROLLER_ID = 15;
    public static final int RIGHT_INTAKE_ROLLER_ID = 25;

    // === CLIMBER ===
    public static final int CLIMBER_RIGHT_ID = 16;
    public static final int CLIMBER_LEFT_ID = 17;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    // === FLYWHEEL KICKER ===
    public static final int FLYWHEEL_KICKER_ID = 20;

    // === HOPPER SENSOR ===
    public static final int HOPPER_SENSOR_ID = 21;

    // === HOPPER ===
    public static final int HOPPER_ROLLER_ID = 22;
    public static final int TWINDEXER_ID = 23;
    // public static final int HOPPER_SENSOR_ID = 25;

    // === HOOD ===
    public static final int HOOD_ID = 24;

    // === LIMELIGHT ===
    public static final String LIMELIGHT_RIGHT = "limelight-right";
    public static final String LIMELIGHT_LEFT = "limelight-left";

    // === CANBUS ===
    public static final CANBus CANBUS = new CANBus("Default Name");

    // === MAXIMUMS ===
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.85);
    public static final AngularVelocity maxAngularVelocity = RevolutionsPerSecond.of(2.5);

    static {
      shotHoodAngleMap.put(6.0, 16.0);
      shotHoodAngleMap.put(5.0, 16.0);

      shotHoodAngleMap.put(4.0, 17.0);

      shotHoodAngleMap.put(3.5, 16.0);
      shotHoodAngleMap.put(3.0, 10.0);
      shotHoodAngleMap.put(2.5, 6.0);
      shotHoodAngleMap.put(2.0, 3.0);
      shotHoodAngleMap.put(1.25, 0.0);

      // === SHOOTING VALUES ===
      shotFlywheelSpeedMap.put(6.0, 2600.0);
      shotFlywheelSpeedMap.put(5.0, 2550.0);

      shotFlywheelSpeedMap.put(4.0, 2500.0);

      shotFlywheelSpeedMap.put(3.5, 2325.0);
      shotFlywheelSpeedMap.put(3.0, 2200.0);
      shotFlywheelSpeedMap.put(2.5, 2100.0);
      shotFlywheelSpeedMap.put(2.0, 2000.0);
      shotFlywheelSpeedMap.put(1.25, 1900.0);

      // ARCED PASSING MAP
      // passHoodAngleMap.put(15.0, 20.0);
      // // passHoodAngleMap.put(5.0, 20.0); // TESTED
      // // passHoodAngleMap.put(4.0, 20.0);
      // // passHoodAngleMap.put(3.0, 20.0); // TESTED
      // // passHoodAngleMap.put(2.5, 20.0); // TESTED
      // // passHoodAngleMap.put(2.0, 20.0);
      // passHoodAngleMap.put(1.0, 20.0);
      // passHoodAngleMap.put(0.0, 20.0);

      // passFlywheelSpeedMap.put(15.0, 4500.0);
      // passFlywheelSpeedMap.put(9.0, 4000.0);
      // passFlywheelSpeedMap.put(7.0, 3500.0);
      // passFlywheelSpeedMap.put(5.0, 3000.0); // TESTED
      // passFlywheelSpeedMap.put(4.0, 2750.0);
      // passFlywheelSpeedMap.put(3.0, 2600.0); // TESTED
      // passFlywheelSpeedMap.put(2.5, 2500.0); // TESTED
      // passFlywheelSpeedMap.put(2.0, 2200.0);
      // passFlywheelSpeedMap.put(1.0, 2000.0);
      // passFlywheelSpeedMap.put(0.0, 2000.0);

      // AGGRESSIVE LOW ANGLE PASS MAP
      passHoodAngleMap.put(12.0, 40.0);
      passHoodAngleMap.put(9.0, 40.0);
      passHoodAngleMap.put(1.0, 40.0);
      passHoodAngleMap.put(0.0, 40.0);

      passFlywheelSpeedMap.put(12.0, 2800.0);
      passFlywheelSpeedMap.put(9.0, 2800.0);
      passFlywheelSpeedMap.put(6.0, 2600.0);
      passFlywheelSpeedMap.put(5.0, 2200.0); // TESTED
      passFlywheelSpeedMap.put(4.0, 2000.0);
      passFlywheelSpeedMap.put(3.0, 1700.0); // TESTED
      passFlywheelSpeedMap.put(2.5, 1500.0); // TESTED
      passFlywheelSpeedMap.put(2.0, 1500.0);
      passFlywheelSpeedMap.put(1.0, 1500.0);
      passFlywheelSpeedMap.put(0.0, 1500.0);

      timeOfFlightMap.put(0.0, 1.05);
      timeOfFlightMap.put(1.75, 1.05);
      timeOfFlightMap.put(2.05, 1.0);
      timeOfFlightMap.put(3.1, 1.05);
      timeOfFlightMap.put(4.8, 1.02);
    }
  }

  public static class CompBotConstants {
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

    public static final double HOOD_OFFSET = 2.0;

    public static final double INDEXER_TO_FLYWHEEL_SECONDS = 0.4;

    public static final double MIN_SHOT_DISTANCE_METERS = 1.25;
    public static final double MAX_SHOT_DISTANCE_METERS = 6.0;

    public static final double MIN_PASS_DISTANCE_METERS = 1.0;
    public static final double MAX_PASS_DISTANCE_METERS = 12.0;

    // === INTAKE ===
    public static final int INTAKE_PIVOT_ID = 14;
    public static final int LEFT_INTAKE_ROLLER_ID = 15;
    public static final int RIGHT_INTAKE_ROLLER_ID = 16;

    // === CLIMBER ===
    public static final int CLIMBER_RIGHT_ID = 17;

    // === FLYWHEEL ===
    public static final int FLYWHEEL_RIGHT_ID = 18;
    public static final int FLYWHEEL_LEFT_ID = 19;

    // === FLYWHEEL KICKER ===
    public static final int FLYWHEEL_KICKER_ID = 20;

    // === HOPPER SENSOR ===
    public static final int HOPPER_SENSOR_ID = 21;

    // === HOPPER ===
    public static final int HOPPER_ROLLER_ID = 22;
    public static final int TWINDEXER_ID = 23;

    // === HOOD ===
    public static final int HOOD_ID = 24;

    // === LIMELIGHT ===
    public static final String LIMELIGHT_RIGHT = "limelight-right";
    public static final String LIMELIGHT_LEFT = "limelight-left";

    // === CANBUS ===
    public static final CANBus CANBUS = new CANBus("Default Name");

    // === MAXIMUMS ===
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.85);
    public static final AngularVelocity maxAngularVelocity = RevolutionsPerSecond.of(2.5);

    static {
      shotHoodAngleMap.put(6.0, 16.0);
      shotHoodAngleMap.put(5.0, 16.0);
      shotHoodAngleMap.put(4.0, 17.0);
      shotHoodAngleMap.put(3.5, 16.0);
      shotHoodAngleMap.put(3.0, 10.0);
      shotHoodAngleMap.put(2.5, 6.0);
      shotHoodAngleMap.put(2.0, 3.0);
      shotHoodAngleMap.put(1.25, 0.0);

      shotFlywheelSpeedMap.put(6.0, 2500.0);
      shotFlywheelSpeedMap.put(5.0, 2450.0);
      shotFlywheelSpeedMap.put(4.0, 2400.0);
      shotFlywheelSpeedMap.put(3.5, 2225.0);
      shotFlywheelSpeedMap.put(3.0, 2100.0);
      shotFlywheelSpeedMap.put(2.5, 2000.0);
      shotFlywheelSpeedMap.put(2.0, 1900.0);
      shotFlywheelSpeedMap.put(1.25, 1800.0);

      passHoodAngleMap.put(12.0, 35.0);
      passHoodAngleMap.put(9.0, 40.0);
      passHoodAngleMap.put(1.0, 40.0);
      passHoodAngleMap.put(0.0, 40.0);

      passFlywheelSpeedMap.put(12.0, 4500.0);
      passFlywheelSpeedMap.put(9.0, 3750.0);
      passFlywheelSpeedMap.put(7.0, 3000.0);
      passFlywheelSpeedMap.put(5.0, 2200.0);
      passFlywheelSpeedMap.put(4.0, 2000.0);
      passFlywheelSpeedMap.put(3.0, 1700.0);
      passFlywheelSpeedMap.put(2.5, 1500.0);
      passFlywheelSpeedMap.put(2.0, 1500.0);
      passFlywheelSpeedMap.put(1.0, 1500.0);
      passFlywheelSpeedMap.put(0.0, 1500.0);

      timeOfFlightMap.put(0.0, 1.05);
      timeOfFlightMap.put(1.75, 1.05);
      timeOfFlightMap.put(2.05, 1.0);
      timeOfFlightMap.put(3.1, 1.05);
      timeOfFlightMap.put(4.8, 1.02);
    }
  }

  public static class SimulationConstants {
    public static final double SIM_TICK_RATE_S = 0.02;

    // === INTAKE ===
    public static final int INTAKE_ROLLER_MOTOR = 30;
    public static final int INTAKE_ROLLER_SENSOR_PORT = 10;
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

    // === MAXIMUMS ===
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.69);
    public static final AngularVelocity maxAngularVelocity = RevolutionsPerSecond.of(4.0);

    // === SHOT CALCULATOR ===
    public static final InterpolatingDoubleTreeMap shotHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passHoodAngleMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shotTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap passTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    public static final double INDEXER_TO_FLYWHEEL_SECONDS = 0.4;

    public static final double MIN_SHOT_DISTANCE_METERS = 0.0;
    public static final double MAX_SHOT_DISTANCE_METERS = 6.0;
    public static final double MIN_PASS_DISTANCE_METERS = 0.0;
    public static final double MAX_PASS_DISTANCE_METERS = 10.0;

    static {
      // === HUB SHOOTING VALUES (based on PracticeBot) ===
      shotHoodAngleMap.put(6.0, 18.0);
      shotHoodAngleMap.put(5.0, 18.0);
      shotHoodAngleMap.put(4.0, 15.0);
      shotHoodAngleMap.put(3.0, 6.0);
      shotHoodAngleMap.put(2.5, 4.0);
      shotHoodAngleMap.put(2.0, 2.0);
      shotHoodAngleMap.put(1.0, 0.0);
      shotHoodAngleMap.put(0.0, 0.0);

      shotFlywheelSpeedMap.put(6.0, 2500.0);
      shotFlywheelSpeedMap.put(5.0, 2500.0);
      shotFlywheelSpeedMap.put(4.0, 2250.0);
      shotFlywheelSpeedMap.put(3.0, 2250.0);
      shotFlywheelSpeedMap.put(2.5, 2150.0);
      shotFlywheelSpeedMap.put(2.0, 2000.0);
      shotFlywheelSpeedMap.put(1.0, 1800.0);
      shotFlywheelSpeedMap.put(0.0, 2000.0);

      shotTimeOfFlightMap.put(0.0, 0.1);
      shotTimeOfFlightMap.put(1.0, 0.15);
      shotTimeOfFlightMap.put(2.0, 0.2);
      shotTimeOfFlightMap.put(3.0, 0.3);
      shotTimeOfFlightMap.put(4.0, 0.4);
      shotTimeOfFlightMap.put(5.0, 0.5);
      shotTimeOfFlightMap.put(6.0, 0.6);

      // === PASSING VALUES (based on PracticeBot, tuned for longer distances) ===
      passHoodAngleMap.put(10.0, 30.0);
      passHoodAngleMap.put(8.0, 26.0);
      passHoodAngleMap.put(6.0, 22.0);
      passHoodAngleMap.put(4.0, 16.0);
      passHoodAngleMap.put(2.0, 10.0);

      passFlywheelSpeedMap.put(10.0, 4500.0);
      passFlywheelSpeedMap.put(8.0, 4000.0);
      passFlywheelSpeedMap.put(6.0, 3500.0);
      passFlywheelSpeedMap.put(4.0, 3000.0);
      passFlywheelSpeedMap.put(2.0, 2500.0);

      passTimeOfFlightMap.put(0.0, 0.15);
      passTimeOfFlightMap.put(2.0, 0.3);
      passTimeOfFlightMap.put(4.0, 0.5);
      passTimeOfFlightMap.put(6.0, 0.7);
      passTimeOfFlightMap.put(8.0, 0.9);
      passTimeOfFlightMap.put(10.0, 1.1);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SerialAddressConstants {
    public static final String WOOD_SERIAL_ADDRESS = "032BE44A";
    public static final String COMP_SERIAL_ADDRESS = "025AE07E";
    public static final String PRACTICE_SERIAL_ADDRESS = "03260AD5";
  }
}
