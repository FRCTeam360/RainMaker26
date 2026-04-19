package frc.robot.subsystems.Shooter.FlywheelKicker;

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
import frc.robot.Constants.CompBotConstants;

public class FlywheelKickerIOCB implements FlywheelKickerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int STALL_CURRENT_LIMIT_AMPS = 60;
  private static final int FREE_CURRENT_LIMIT_AMPS = 50;

  // Spinup config - bang-bang maximum acceleration (extremely high kP drives full output)
  private static final double SPINUP_KP = 999999.0;
  private static final double SPINUP_KI = 0.0;
  private static final double SPINUP_KD = 0.0;

  // Hold config - smooth setpoint maintenance with tuned PID
  private static final double HOLD_KP = 0.0019;
  private static final double HOLD_KI = 0.0;
  private static final double HOLD_KD = 0.0;

  // Feedforward (shared across all slots)
  private static final double KV = 0.0017;
  private static final double KS = 0.04;

  private static final double MAX_NEGATIVE_OUTPUT = 0.0;
  private static final double MAX_POSITIVE_OUTPUT = 1.0;

  /** Creates a new FlywheelKickerIOCB. */
  private final SparkFlex flywheelKickerMotor;

  private final RelativeEncoder encoder;
  private final SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  public FlywheelKickerIOCB() {
    flywheelKickerMotor = new SparkFlex(CompBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);
    encoder = flywheelKickerMotor.getEncoder();

    // Configure base motor settings
    sparkFlexConfig.idleMode(IdleMode.kCoast);
    sparkFlexConfig.inverted(false);
    sparkFlexConfig.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);
    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.quadratureMeasurementPeriod(25);
    sparkFlexConfig.encoder.quadratureAverageDepth(8);

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
  }

  @Override
  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelKickerMotor.getOutputCurrent();
    inputs.supplyCurrent = 0;
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelKickerMotor.getBusVoltage() * flywheelKickerMotor.getAppliedOutput();
    inputs.sensorProximity = 0.0;
    inputs.sensorActivated = false;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    flywheelKickerMotor.set(dutyCycle);
  }

  @Override
  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setSpinupVelocityControl(double rpm) {
    // Use Slot 0 for aggressive spinup
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHoldVelocityControl(double rpm) {
    // Use Slot 1 for smooth hold
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
}
