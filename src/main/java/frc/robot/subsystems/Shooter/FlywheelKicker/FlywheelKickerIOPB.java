package frc.robot.subsystems.Shooter.FlywheelKicker;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class FlywheelKickerIOPB implements FlywheelKickerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int CURRENT_LIMIT_AMPS = 40;
  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double FF_KV = 0.0021;
  private static final double FF_KS = 0.04;
  private static final double MIN_SIGNAL_STRENGTH = 2000; // unknown unit
  private static final double PROXIMITY_THRESHOLD_METERS = 0.1;

  /** Creates a new FlywheelKickerIOPB. */
  private final SparkFlex flywheelKickerMotor =
      new SparkFlex(Constants.PracticeBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = flywheelKickerMotor.getEncoder();
  private final SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  // FIXME: reimplement when CANRange is added to practice bot
  // private final CANrange canSensor =
  //     new CANrange(Constants.PracticeBotConstants.FLYWHEEL_KICKER_SENSOR_ID,
  // Constants.RIO_CANBUS);

  // private final StatusSignal<Distance> distanceSignal;
  // private final StatusSignal<Boolean> isDetectedSignal;

  public FlywheelKickerIOPB() {
    sparkFlexConfig.idleMode(IdleMode.kBrake);
    sparkFlexConfig.inverted(false);
    sparkFlexConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    sparkFlexConfig.closedLoop.p(KP).i(KI).d(KD);
    sparkFlexConfig.closedLoop.feedForward.kV(FF_KV).kS(FF_KS);

    flywheelKickerMotor.configure(
        sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = flywheelKickerMotor.getClosedLoopController();

    CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
    sensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH;
    sensorConfig.ProximityParams.ProximityThreshold = PROXIMITY_THRESHOLD_METERS;
    sensorConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRangeUserFreq);
    // canSensor.getConfigurator().apply(sensorConfig);

    // distanceSignal = canSensor.getDistance();
    // isDetectedSignal = canSensor.getIsDetected();

    // BaseStatusSignal.setUpdateFrequencyForAll(50, distanceSignal, isDetectedSignal);
    // canSensor.optimizeBusUtilization();
  }

  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelKickerMotor.getOutputCurrent();
    inputs.supplyCurrent = 0;
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelKickerMotor.getBusVoltage() * flywheelKickerMotor.getAppliedOutput();
    inputs.sensorProximity = 0.0;
    // inputs.sensorProximity = canSensor.getDistance().getValueAsDouble();
    inputs.sensorActivated = false;
    // inputs.sensorActivated = canSensor.getIsDetected().getValue();
  }

  public void setDutyCycle(double dutyCycle) {
    flywheelKickerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
