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
import frc.robot.Constants.PracticeBotConstants;

public class ClimberIOPB implements ClimberIO {

  private final SparkMax leftClimberMotor =
      new SparkMax(PracticeBotConstants.CLIMBER_ID, MotorType.kBrushless);
  private final SparkMax rightClimberMotor = 
      new SparkMax(PracticeBotConstants.CLIMBER_ID, MotorType.kBrushless);
  
  private final RelativeEncoder leftClimberEncoder = leftClimberMotor.getEncoder();
  private final RelativeEncoder rightClimberEncoder = rightClimberMotor.getEncoder();

  private final double positionConversionFactor = 1.0;
  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static class UnloadedConstants {
      static final double kP = 1.0;
      static final double kI = 0.0001;
      static final double kD = 0;

  }

  private static class LoadedConstants {
        static final double kP = 1.0;
        static final double kI = 0.0001;
        static final double kD = 0;
        static final double kG = -0.03;
  }

  /** Creates a new ClimberIOPB. */
  public ClimberIOPB() {
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    EncoderConfig leftEncoderConfig = new EncoderConfig();
    EncoderConfig rightEncoderConfig = new EncoderConfig();

    closedLoopConfig.pid(kP, kI, kD);
    leftEncoderConfig.positionConversionFactor(positionConversionFactor);
    rightEncoderConfig.positionConversionFactor(positionConversionFactor);

    leftConfig.inverted(true);
    leftConfig.apply(leftEncoderConfig);
    leftConfig.apply(closedLoopConfig);

    rightConfig.inverted(false);
    rightConfig.apply(rightEncoderConfig);
    rightConfig.apply(closedLoopConfig);

    leftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setLeftDutyCycle(double dutyCycle) {
    leftClimberMotor.set(dutyCycle);
  }

  public void setRightDutyCycle(double dutyCycle) {
    rightClimberMotor.set(dutyCycle);
  }

  public void setLeftPosition(double position) {
    leftClimberMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  } 

  public void setRightPosition(double position) {
    rightClimberMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
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
