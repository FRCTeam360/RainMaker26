// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project..

package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

import frc.robot.Constants.PracticeBotConstants;

public class ClimberIOPB implements ClimberIO {

  private final SparkMax leftClimberMotor =
      new SparkMax(PracticeBotConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
  private final SparkMax rightClimberMotor =
      new SparkMax(PracticeBotConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

  private final RelativeEncoder leftClimberEncoder = leftClimberMotor.getEncoder();
  private final RelativeEncoder rightClimberEncoder = rightClimberMotor.getEncoder();

  private static final double POSITION_CONVERSION_FACTOR = 1.0;
  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  public void zeroBoth() {
    leftClimberEncoder.setPosition(0.0);
    rightClimberEncoder.setPosition(0.0);
  }

  /** Creates a new ClimberIOPB. */
  public ClimberIOPB() {
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    EncoderConfig leftEncoderConfig = new EncoderConfig();
    EncoderConfig rightEncoderConfig = new EncoderConfig();

    closedLoopConfig.pid(kP, kI, kD);
    leftEncoderConfig.positionConversionFactor(POSITION_CONVERSION_FACTOR);
    rightEncoderConfig.positionConversionFactor(POSITION_CONVERSION_FACTOR);

    leftConfig.inverted(true);
    leftConfig.apply(leftEncoderConfig);
    leftConfig.apply(closedLoopConfig);
    leftConfig.limitSwitch.forwardLimitSwitchPosition(0.0).forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    rightConfig.inverted(false);
    rightConfig.apply(rightEncoderConfig);
    rightConfig.apply(closedLoopConfig);
    rightConfig.limitSwitch.forwardLimitSwitchPosition(0.0).forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);


    leftClimberMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimberMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setLeftDutyCycle(double dutyCycle) {
    leftClimberMotor.set(dutyCycle);
  }

  public void setRightDutyCycle(double dutyCycle) {
    rightClimberMotor.set(dutyCycle);
  }

  public void setLeftPosition(double position) {
    leftClimberMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
  }

  public void setRightPosition(double position) {
    rightClimberMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeftDutyCycle = leftClimberMotor.getAppliedOutput();
    inputs.climberRightDutyCycle = rightClimberMotor.getAppliedOutput();
    inputs.climberLeftPosition = leftClimberEncoder.getPosition();
    inputs.climberRightPosition = rightClimberEncoder.getPosition();
    inputs.climberLeftVelocity = leftClimberEncoder.getVelocity();
    inputs.climberRightVelocity = rightClimberEncoder.getVelocity();
    inputs.climberLeftCurrent = leftClimberMotor.getOutputCurrent();
    inputs.climberRightCurrent = rightClimberMotor.getOutputCurrent();
    inputs.climberLeftTemp = leftClimberMotor.getMotorTemperature();
    inputs.climberRightTemp = rightClimberMotor.getMotorTemperature();
  }
}
