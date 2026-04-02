// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project..

package frc.robot.subsystems.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CompBotConstants;

public class ClimberIOCB implements ClimberIO {

  private static final double GEAR_RATIO = 1.0;

  // Motor rotations to inches assumes perfect wrapping around a 0.5 in diameter hex shaft
  // TODO measure this empirically to get a better ratio
  private static final double MOTOR_ROTATIONS_TO_INCHES = 1.732;

  private final SparkMax leftClimberMotor;
  private final SparkMax rightClimberMotor;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  private static final double REVERSE_SOFT_LIMIT_INCHES = 0.0;
  private static final double FORWARD_SOFT_LIMIT_INCHES = 36.0;

  public void zeroBoth() {
    leftClimberEncoder.setPosition(0.0);
    rightClimberEncoder.setPosition(0.0);
  }

  /** Creates a new ClimberIOCB. */
  public ClimberIOCB() {
    this(CompBotConstants.CLIMBER_LEFT_ID, CompBotConstants.CLIMBER_RIGHT_ID);
  }

  protected ClimberIOCB(int climberLeftId, int climberRightId) {
    leftClimberMotor = new SparkMax(climberLeftId, MotorType.kBrushless);
    rightClimberMotor = new SparkMax(climberRightId, MotorType.kBrushless);
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    EncoderConfig leftEncoderConfig = new EncoderConfig();
    EncoderConfig rightEncoderConfig = new EncoderConfig();

    closedLoopConfig.pid(kP, kI, kD);
    leftEncoderConfig.positionConversionFactor(GEAR_RATIO);
    leftEncoderConfig.velocityConversionFactor(GEAR_RATIO);

    rightEncoderConfig.positionConversionFactor(GEAR_RATIO);
    rightEncoderConfig.velocityConversionFactor(GEAR_RATIO);

    leftConfig.inverted(true);
    leftConfig.apply(leftEncoderConfig);
    leftConfig.apply(closedLoopConfig);
    leftConfig
        .limitSwitch
        .forwardLimitSwitchPosition(FORWARD_SOFT_LIMIT_INCHES / MOTOR_ROTATIONS_TO_INCHES)
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
    leftConfig
        .limitSwitch
        .reverseLimitSwitchPosition(REVERSE_SOFT_LIMIT_INCHES / MOTOR_ROTATIONS_TO_INCHES)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    rightConfig.inverted(false);
    rightConfig.apply(rightEncoderConfig);
    rightConfig.apply(closedLoopConfig);
    rightConfig
        .limitSwitch
        .forwardLimitSwitchPosition(FORWARD_SOFT_LIMIT_INCHES)
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
    rightConfig
        .limitSwitch
        .reverseLimitSwitchPosition(REVERSE_SOFT_LIMIT_INCHES)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

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
    leftClimberMotor
        .getClosedLoopController()
        .setSetpoint(position / MOTOR_ROTATIONS_TO_INCHES, ControlType.kPosition);
  }

  public void setRightPosition(double position) {
    rightClimberMotor
        .getClosedLoopController()
        .setSetpoint(position / MOTOR_ROTATIONS_TO_INCHES, ControlType.kPosition);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeftDutyCycle = leftClimberMotor.getAppliedOutput();
    inputs.climberRightDutyCycle = rightClimberMotor.getAppliedOutput();
    inputs.climberLeftPosition = leftClimberEncoder.getPosition() * MOTOR_ROTATIONS_TO_INCHES;
    inputs.climberRightPosition = rightClimberEncoder.getPosition() * MOTOR_ROTATIONS_TO_INCHES;
    inputs.climberLeftVelocity =
        leftClimberEncoder.getVelocity() * MOTOR_ROTATIONS_TO_INCHES / 60.0;
    inputs.climberRightVelocity =
        rightClimberEncoder.getVelocity() * MOTOR_ROTATIONS_TO_INCHES / 60.0;
    inputs.climberLeftCurrent = leftClimberMotor.getOutputCurrent();
    inputs.climberRightCurrent = rightClimberMotor.getOutputCurrent();
    inputs.climberLeftTemp = leftClimberMotor.getMotorTemperature();
    inputs.climberRightTemp = rightClimberMotor.getMotorTemperature();
  }
}
