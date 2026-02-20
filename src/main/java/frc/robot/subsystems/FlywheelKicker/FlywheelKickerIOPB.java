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
    sparkMaxConfig.smartCurrentLimit(40);

    sparkMaxConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkMaxConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    sparkMaxConfig.closedLoop.p(0.0002).i(0.0).d(0.0);
    sparkMaxConfig.closedLoop.feedForward.kV(0.0021).kS(0.04);

    flywheelkickerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = flywheelkickerMotor.getClosedLoopController();

    CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
    sensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // unknown unit
    sensorConfig.ProximityParams.ProximityThreshold = 0.1; // meters
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
