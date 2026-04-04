// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakePivotIOCB implements IntakePivotIO {
  // TODO: UPDATE GEAR RATIO
  private static final double GEAR_RATIO = 97.5;
  // 80-16 * 52-16 * 40-20 * 36-12

  private static final double KP = 175.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KA = 0.0;
  private static final double KG = 0.15;
  private static final double KS = 0.35;
  private static final double KV = 0.0;

  private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  private static final double MOTION_MAGIC_ACCELERATION_RPS2 = 3.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY_RPS = 1.5;
  private static final double MOTION_MAGIC_JERK_RPS3 = 1750.0;

  private static final double FORWARD_SOFT_LIMIT_DEGREES = 97.0;
  private static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0;

  private static final double PEAK_FORWARD_VOLTAGE = 12.0;
  private static final double PEAK_REVERSE_VOLTAGE = -12.0;

  private final TalonFX intakePivot;

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;

  /** Creates a new IntakePivotIOCB. */
  public IntakePivotIOCB() {
    this(Constants.CompBotConstants.INTAKE_PIVOT_ID, Constants.CompBotConstants.CANBUS);
  }

  protected IntakePivotIOCB(int intakePivotId, CANBus canBus) {
    intakePivot = new TalonFX(intakePivotId, canBus);

    // FIXME: NEUTRAL MODE BRAKE
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = KA;
    slot0Configs.kD = KD;
    slot0Configs.kG = KG;
    slot0Configs.kI = KI;
    slot0Configs.kP = KP;
    slot0Configs.kS = KS;
    slot0Configs.kV = KV;

    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY_RPS;
    motionMagicConfigs.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION_RPS2;
    motionMagicConfigs.MotionMagicJerk = MOTION_MAGIC_JERK_RPS3;

    config.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;
    // TODO: GRAVITY TYPE COSINE / ARM
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
            Units.degreesToRotations(FORWARD_SOFT_LIMIT_DEGREES))
        .withReverseSoftLimitThreshold(Units.degreesToRotations(REVERSE_SOFT_LIMIT_DEGREES))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);
    config.MotorOutput.NeutralMode = neutralMode;

    intakePivot.getConfigurator().apply(config, 0.050);

    positionSignal = intakePivot.getPosition();
    velocitySignal = intakePivot.getVelocity();
    statorCurrentSignal = intakePivot.getStatorCurrent();
    supplyCurrentSignal = intakePivot.getSupplyCurrent();
    motorVoltageSignal = intakePivot.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionSignal,
        velocitySignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        motorVoltageSignal);
    intakePivot.optimizeBusUtilization();
  }

  public void setZero() {
    intakePivot.setPosition(0.0);
  }

  /**
   * Sets the intake pivot position.
   *
   * @param positionDegrees target position in degrees
   */
  public void setPosition(double positionDegrees) {
    intakePivot.setControl(
        motionMagicPosition.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  public void setPositionSmooth(double positionDegrees) {
    intakePivot.setControl(
        motionMagicPosition.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  public void setPositionAggressive(double positionDegrees) {
    intakePivot.setControl(positionVoltage.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  public void setDutyCycle(double value) {
    intakePivot.setControl(dutyCycleOut.withOutput(value));
  }

  public void enableBrakeMode() {
    neutralMode = NeutralModeValue.Brake;
    intakePivot.setNeutralMode(NeutralModeValue.Brake);
  }

  public void disableBrakeMode() {
    neutralMode = NeutralModeValue.Coast;
    intakePivot.setNeutralMode(NeutralModeValue.Coast);
  }

  // TODO: ASK ELECTRICAL FOR A ZEROING BUTTON OR AN ABSOLUTE ENCODER

  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        motorVoltageSignal);
    inputs.position = Units.rotationsToDegrees(positionSignal.getValueAsDouble());
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.velocity = Units.rotationsToDegrees(velocitySignal.getValueAsDouble());
    inputs.voltage = motorVoltageSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.brakeMode = neutralMode == NeutralModeValue.Brake;
  }
}
