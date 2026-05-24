package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

/**
 * Facade that exposes the active robot's Tuner X swerve constants. Selects the appropriate hardware
 * configuration (WoodBot, PracticeBot, CompBot) based on {@link Constants#getRobotType()}.
 *
 * <p>All fields follow the naming conventions expected by the AdvantageKit swerve template so that
 * drive IO implementations can reference {@code TunerConstants.FrontLeft}, etc. directly.
 */
public final class TunerConstants {
  private TunerConstants() {}

  public static final CANBus kCANBus;
  public static final LinearVelocity kSpeedAt12Volts;
  public static final SwerveDrivetrainConstants DrivetrainConstants;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  static {
    switch (Constants.getRobotType()) {
      case WOODBOT:
      case SIM:
      case REPLAY:
        kCANBus = WoodBotDrivetrain.kCANBus;
        kSpeedAt12Volts = WoodBotDrivetrain.kSpeedAt12Volts;
        DrivetrainConstants = WoodBotDrivetrain.DrivetrainConstants;
        FrontLeft = WoodBotDrivetrain.FrontLeft;
        FrontRight = WoodBotDrivetrain.FrontRight;
        BackLeft = WoodBotDrivetrain.BackLeft;
        BackRight = WoodBotDrivetrain.BackRight;
        break;
      case PRACTICEBOT:
        kCANBus = PracticeBotDrivetrain.kCANBus;
        kSpeedAt12Volts = PracticeBotDrivetrain.kSpeedAt12Volts;
        DrivetrainConstants = PracticeBotDrivetrain.DrivetrainConstants;
        FrontLeft = PracticeBotDrivetrain.FrontLeft;
        FrontRight = PracticeBotDrivetrain.FrontRight;
        BackLeft = PracticeBotDrivetrain.BackLeft;
        BackRight = PracticeBotDrivetrain.BackRight;
        break;
      case COMPBOT:
      default:
        kCANBus = CompBotDrivetrain.kCANBus;
        kSpeedAt12Volts = CompBotDrivetrain.kSpeedAt12Volts;
        DrivetrainConstants = CompBotDrivetrain.DrivetrainConstants;
        FrontLeft = CompBotDrivetrain.FrontLeft;
        FrontRight = CompBotDrivetrain.FrontRight;
        BackLeft = CompBotDrivetrain.BackLeft;
        BackRight = CompBotDrivetrain.BackRight;
        break;
    }
  }
}
