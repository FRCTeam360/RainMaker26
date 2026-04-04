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

  private final SparkMax rightClimberMotor;

  private final RelativeEncoder rightClimberEncoder;

  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  private static final double REVERSE_SOFT_LIMIT_INCHES = 0.0;
  private static final double FORWARD_SOFT_LIMIT_INCHES = 36.0;

  public void zeroBoth() {
    rightClimberEncoder.setPosition(0.0);
  }

  /** Creates a new ClimberIOCB. */
  public ClimberIOCB() {
    rightClimberMotor = new SparkMax(CompBotConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);
    rightClimberEncoder = rightClimberMotor.getEncoder();

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    EncoderConfig rightEncoderConfig = new EncoderConfig();

    closedLoopConfig.pid(kP, kI, kD);

    rightEncoderConfig.positionConversionFactor(GEAR_RATIO);
    rightEncoderConfig.velocityConversionFactor(GEAR_RATIO);

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

    rightClimberMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setLeftDutyCycle(double dutyCycle) {}

  public void setRightDutyCycle(double dutyCycle) {
    rightClimberMotor.set(dutyCycle);
  }

  public void setLeftPosition(double position) {}

  public void setRightPosition(double position) {
    rightClimberMotor
        .getClosedLoopController()
        .setSetpoint(position / MOTOR_ROTATIONS_TO_INCHES, ControlType.kPosition);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberRightDutyCycle = rightClimberMotor.getAppliedOutput();
    inputs.climberRightPosition = rightClimberEncoder.getPosition() * MOTOR_ROTATIONS_TO_INCHES;
    inputs.climberRightVelocity =
        rightClimberEncoder.getVelocity() * MOTOR_ROTATIONS_TO_INCHES / 60.0;
    inputs.climberRightCurrent = rightClimberMotor.getOutputCurrent();
    inputs.climberRightTemp = rightClimberMotor.getMotorTemperature();
  }
}
