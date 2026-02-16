// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakePivotIOPB implements IntakePivotIO {
  private final TalonFX intakePivot =
      new TalonFX(Constants.WoodBotConstants.INTAKE_PIVOT_ID, Constants.WoodBotConstants.CANBUS);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private final SoftLimitConfig softLimitConfig = new SoftLimitConfig();
  private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  // do not call --> will result in null pointer
  private DigitalInput zeroButton;
  private DigitalInput brakeButton;

  private boolean zeroPrev = false;
  private boolean brakePrev = false;

  // TODO: UPDATE GEAR RATIO
  private final double GEAR_RATIO = 360.0 / 60.0;

  // TODO: ADD CONVERSION FACTOR TO GET MECHANISM INTO DEGREES

  /** Creates a new IntakePivotIOWB. */
  public IntakePivotIOPB() {
    intakePivot.getConfigurator().apply(config);
    intakePivot.setNeutralMode(NeutralModeValue.Brake);

    final double motionMagicAcceleration = 400.0; // rotations per second squared
    final double motionMagicCruiseVelocity = 85.0; // rotations per second
    final double motionMagicCruiseJerk = 1750.0; // rotations per second cubed

    final double forwardLimit =
        178.0; // TODO: make sure these are correct for prac bots; units are in degrees
    final double reverseLimit = 0.0; // 29.5, units are in degrees

    currentLimitConfig.StatorCurrentLimit = 120.0;
    currentLimitConfig.SupplyCurrentLimit = 60.0;

    softLimitConfig.forwardSoftLimitEnabled(true);
    softLimitConfig.forwardSoftLimit(0.0);
    softLimitConfig.reverseSoftLimitEnabled(true);
    softLimitConfig.reverseSoftLimit(50.0);

    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
    final double kA = 0.0;
    final double kG = 0.0;
    final double kS = 0.0;
    final double kV = 0.0;

    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = 12.0;

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(forwardLimit)
        .withReverseSoftLimitThreshold(reverseLimit)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    config.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        .withMotionMagicJerk(motionMagicCruiseJerk);

    intakePivot.getConfigurator().apply(config, 0.050);
  }

  public void setZero() {
    intakePivot.setPosition(0.0);
  }

  public void setPosition(double position) {
    intakePivot.setControl(motionMagicPosition.withPosition(position));
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

  public boolean isBrakeMode() {
    return neutralMode == NeutralModeValue.Brake;
  }

  // TODO: ASK ELECTRICAL FOR A ZEROING BUTTON OR AN ABSOLUTE ENCODER
  private boolean getRawZeroButton() {
    return !this.zeroButton.get();
  }

  public boolean getZeroButton() {
    boolean zeroCurr = getRawZeroButton();
    boolean risingEdge = zeroCurr && !zeroPrev;
    zeroPrev = zeroCurr;
    return risingEdge;
  }

  private boolean getRawBrakeButton() {
    return !this.brakeButton.get();
  }

  public boolean getBrakeButton() {
    boolean brakeCurr = getRawBrakeButton();
    boolean risingEdge = brakeCurr && !brakePrev;
    brakePrev = brakeCurr;
    return risingEdge;
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = intakePivot.getPosition().getValueAsDouble();
    inputs.statorCurrent = intakePivot.getStatorCurrent().getValueAsDouble();
    inputs.velocity = intakePivot.getVelocity().getValueAsDouble();
    inputs.voltage = intakePivot.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = intakePivot.getSupplyCurrent().getValueAsDouble();
  }
}
