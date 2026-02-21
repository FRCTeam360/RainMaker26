package frc.robot.subsystems.FlywheelKicker;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class FlywheelKickerIOPB implements FlywheelKickerIO {
  private static final double GEAR_RATIO = 1.0; // FIXME: set actual gear ratio
  private static final int CURRENT_LIMIT_AMPS = 40;
  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double FF_KV = 0.0021;
  private static final double FF_KS = 0.04;
  private static final double MIN_SIGNAL_STRENGTH = 2000; // unknown unit
  private static final double PROXIMITY_THRESHOLD_METERS = 0.1;

  /** Creates a new FlywheelKickerIOPB. */
  private final SparkMax flywheelkickerMotor =
      new SparkMax(Constants.PracticeBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = flywheelkickerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController closedLoopController;

  private final CANrange canSensor =
      new CANrange(Constants.PracticeBotConstants.FLYWHEEL_KICKER_SENSOR_ID, Constants.RIO_CANBUS);

  public FlywheelKickerIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    sparkMaxConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

    sparkMaxConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkMaxConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    sparkMaxConfig.closedLoop.p(KP).i(KI).d(KD);
    sparkMaxConfig.closedLoop.feedForward.kV(FF_KV).kS(FF_KS);

    flywheelkickerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = flywheelkickerMotor.getClosedLoopController();

    CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
    sensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH;
    sensorConfig.ProximityParams.ProximityThreshold = PROXIMITY_THRESHOLD_METERS;
    sensorConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRangeUserFreq);
    canSensor.getConfigurator().apply(sensorConfig);
  }

  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelkickerMotor.getOutputCurrent();
    inputs.supplyCurrent = 0;
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelkickerMotor.getBusVoltage() * flywheelkickerMotor.getAppliedOutput();
    inputs.sensorProximity = canSensor.getDistance().getValueAsDouble();
    inputs.sensorActivated = canSensor.getIsDetected().getValue();
  }

  public void setDutyCycle(double dutyCycle) {
    flywheelkickerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
