package frc.robot.subsystems.Shooter.FlywheelKicker;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class FlywheelKickerIOPB implements FlywheelKickerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int STALL_CURRENT_LIMIT_AMPS = 60;
  private static final int FREE_CURRENT_LIMIT_AMPS = 50;

  // Spinup config - bang-bang maximum acceleration (extremely high kP drives full output)
  private static final double SPINUP_KP = 999999.0;
  private static final double SPINUP_KI = 0.0;
  private static final double SPINUP_KD = 0.0;

  // Hold config - smooth setpoint maintenance with tuned PID
  private static final double HOLD_KP = 0.0010;
  private static final double HOLD_KI = 0.0;
  private static final double HOLD_KD = 0.0;

  // Feedforward (shared across all slots)
  private static final double KV = 0.0012;
  private static final double KS = 0.04;

  private static final double MIN_SIGNAL_STRENGTH = 2000; // unknown unit
  private static final double PROXIMITY_THRESHOLD_METERS = 0.1;
  private static final double MAX_NEGATIVE_OUTPUT = 0.0;
  private static final double MAX_POSITIVE_OUTPUT = 1.0;

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
    // Configure base motor settings
    sparkFlexConfig.idleMode(IdleMode.kCoast);
    sparkFlexConfig.inverted(false);
    sparkFlexConfig.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);
    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);
    // sparkFlexConfig.encoder.uvwMeasurementPeriod(10);
    // sparkFlexConfig.encoder.uvwAverageDepth(2);

    // Configure Slot 0: Spinup - aggressive acceleration
    sparkFlexConfig
        .closedLoop
        .p(SPINUP_KP, ClosedLoopSlot.kSlot0)
        .i(SPINUP_KI, ClosedLoopSlot.kSlot0)
        .d(SPINUP_KD, ClosedLoopSlot.kSlot0);
    sparkFlexConfig
        .closedLoop
        .feedForward
        .kV(KV, ClosedLoopSlot.kSlot1)
        .kS(KS, ClosedLoopSlot.kSlot1);
    sparkFlexConfig.closedLoop.outputRange(
        MAX_NEGATIVE_OUTPUT, MAX_POSITIVE_OUTPUT, ClosedLoopSlot.kSlot0);

    // Configure Slot 1: Hold - smooth setpoint maintenance
    sparkFlexConfig
        .closedLoop
        .p(HOLD_KP, ClosedLoopSlot.kSlot1)
        .i(HOLD_KI, ClosedLoopSlot.kSlot1)
        .d(HOLD_KD, ClosedLoopSlot.kSlot1);
    // Feedforward is shared across all slots, already set above
    sparkFlexConfig.closedLoop.outputRange(
        MAX_NEGATIVE_OUTPUT, MAX_POSITIVE_OUTPUT, ClosedLoopSlot.kSlot1);

    // Apply configuration once at startup
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

  public void setSpinupVelocityControl(double rpm) {
    // Use Slot 0 for aggressive spinup
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setHoldVelocityControl(double rpm) {
    // Use Slot 1 for smooth hold
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
}
