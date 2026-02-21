// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

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
  public static final double SPINUP_SHOOTING_FLYWHEEL_RPM = 3250.0;

  public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.69);

  public static final AngularVelocity maxAngularVelocity = RevolutionsPerSecond.of(4.0);

  public static final int IMU_MODE_EXTERNAL_ONLY = 0;

  public static final int IMU_MODE_EXTERNAL_SEED = 1;

  public static final int IMU_MODE_INTERNAL_ONLY = 2;

  public static final int IMU_MODE_INTERNAL_MT1_ASSIST = 3;

  public static final int IMU_MODE_INTERNAL_EXTERNAL_ASSIST = 4;

  public static final double IMU_ASSIST_ALPHA = 0.001;

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

  }

  public static class PracticeBotConstants {

    // FIXME: update these values TO REAL VALUES
    public static final int INTAKE_ID = 71;
    public static final int INTAKE_PIVOT_ID = 72;

    public static final int INDEXER_SENSOR_ID = 73;
    public static final int INDEXER_ID = 75;

    public static final int FLYWHEEL_KICKER_ID = 76;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 77;

    public static final int FLYWHEEL_RIGHT_ID = 78;
    public static final int FLYWHEEL_LEFT_ID = 79;

    public static final int HOOD_ID = 80;

    public static final int HOPPER_ROLLER_ID = 0;

    public static final String LIMELIGHT = "limelight";

    public static final CANBus CANBUS = new CANBus("Default Name");
  }

  public static class PracticeBotConstants {

    // FIXME: update these values TO REAL VALUES
    public static final int INTAKE_ID = 71;
    public static final int INTAKE_PIVOT_ID = 72;

    public static final int INDEXER_SENSOR_ID = 73;
    public static final int INDEXER_ID = 75;

    public static final int FLYWHEEL_KICKER_ID = 76;
    public static final int FLYWHEEL_KICKER_SENSOR_ID = 77;

    public static final int FLYWHEEL_RIGHT_ID = 78;
    public static final int FLYWHEEL_LEFT_ID = 79;

    public static final int HOOD_ID = 80;

    public static final int HOPPER_ROLLER_ID = 0;

    public static final String LIMELIGHT = "limelight";

    public static final CANBus CANBUS = new CANBus("Default Name");
  }

  public static class SimulationConstants {
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

    // === HOOD ===
    public static final int HOOD_MOTOR = 34;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SerialAddressConstants {
    public static final String WOOD_SERIAL_ADDRESS = "032BE44A";
    // FIXME: Enter the practice bot roboRIO serial address
    public static final String PRACTICE_SERIAL_ADDRESS = "WRONG_SERIAL_ADDRESS";
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
    return Constants.RobotType.WOODBOT;
  }
}
