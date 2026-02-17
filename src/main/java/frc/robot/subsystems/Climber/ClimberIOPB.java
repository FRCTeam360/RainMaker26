// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: there is no practice bot constants setup.

package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.WoodBotConstants;

public class ClimberIOPB implements ClimberIO {

  private final SparkMax leftClimberMotor =
      new SparkMax(WoodBotConstants.CLIMBER_ID, MotorType.kBrushless);
  private final SparkMax rightClimberMotor = 
      new SparkMax(WoodBotConstants.CLIMBER_ID, MotorType.kBrushless);
  
  private final RelativeEncoder leftClimberEncoder = leftClimberMotor.getEncoder();
  private final RelativeEncoder rightClimberEncoder = rightClimberMotor.getEncoder();

  private final double kP = 0.2;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final double positionConversionFactor = 1.0;
  private final SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new ClimberIOPB. */
  public ClimberIOPB() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    config.apply(closedLoopConfig);
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    config.apply(encoderConfig);
    leftClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    leftClimberMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    leftClimberMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
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
