// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakePivotIOPB implements IntakePivotIO {
  // TODO: UPDATE GEAR RATIO
  private static final double GEAR_RATIO = 1.0; // was 60.0 before ice cream social

  private static final double KP = 3.0;
  private static final double KI = 0.0;
  private static final double KD = 0.4;
  private static final double KA = 0.0;
  private static final double KG = 3.0;
  private static final double KS = 7.0;
  private static final double KV = 0.0;

  private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;

  private static final double MOTION_MAGIC_ACCELERATION_RPS2 = 200.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY_RPS = 40.0;
  private static final double MOTION_MAGIC_JERK_RPS3 = 1750.0;

  private static final double FORWARD_SOFT_LIMIT_DEGREES =
      178.0; // TODO: make sure these are correct for prac bot
  private static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0; // 29.5

  private static final double PEAK_FORWARD_VOLTAGE = 12.0;
  private static final double PEAK_REVERSE_VOLTAGE = -12.0;

  private final TalonFX intakePivot =
      new TalonFX(
          Constants.PracticeBotConstants.INTAKE_PIVOT_ID, Constants.PracticeBotConstants.CANBUS);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final MotionMagicTorqueCurrentFOC motionMagicPosition =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  /** Creates a new IntakePivotIOPB. */
  public IntakePivotIOPB() {
    intakePivot.setNeutralMode(NeutralModeValue.Brake);

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

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
            Units.degreesToRotations(FORWARD_SOFT_LIMIT_DEGREES))
        .withReverseSoftLimitThreshold(Units.degreesToRotations(REVERSE_SOFT_LIMIT_DEGREES))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    intakePivot.getConfigurator().apply(config, 0.050);
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
    inputs.position = Units.rotationsToDegrees(intakePivot.getPosition().getValueAsDouble());
    inputs.statorCurrent = intakePivot.getStatorCurrent().getValueAsDouble();
    inputs.velocity = Units.rotationsToDegrees(intakePivot.getVelocity().getValueAsDouble());
    inputs.voltage = intakePivot.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = intakePivot.getSupplyCurrent().getValueAsDouble();
    inputs.brakeMode = neutralMode == NeutralModeValue.Brake;
  }
}
