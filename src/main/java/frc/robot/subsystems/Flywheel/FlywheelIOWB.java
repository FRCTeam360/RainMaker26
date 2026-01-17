// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WoodBotConstants;
public class FlywheelIOWB implements FlywheelIO {

  // need motor vvvvvv
  private final TalonFX leftMotor = new TalonFX(WoodBotConstants.FLYWHEEL_LEFT_ID, WoodBotConstants.CANBUS_NAME);
  private final TalonFX rightMotor = new TalonFX(WoodBotConstants.FLYWHEEL_RIGHT_ID, WoodBotConstants.CANBUS_NAME);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

  public void setDutyCycle(double dutyCycle) {
    leftMotor.set(dutyCycle);
    rightMotor.set(dutyCycle);
  }

  @Override
  public void setRPM(double rpm, ControlType kvelocity) 
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 0.0;
    double kV = 0.0;
    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    final double motionMagicCruiseVelocity = 0.0;
    final double motionMagicAcceleration = 0.0; 
    final double motionMagicCruiseJerk = 0.0;
    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());
    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;
     config
        .MotionMagic
        .withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        .withMotionMagicJerk(motionMagicCruiseJerk);
        config.MotorOutput = outputConfigs;
      leftMotor.setNeutralMode(NeutralModeValue.Brake);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    leftMotor.getConfigurator().apply(config, 0.0);

    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightMotor.getConfigurator().apply(config, 0.0);
    
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
  public void updateInputs(FlywheelIOInputs inputs){
    inputs.flywheelStatorCurrents[0] = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelStatorCurrents[1] = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.flywheelPositions[0] = leftMotor.getPosition().getValueAsDouble();
    inputs.flywheelPositions[1] = rightMotor.getPosition().getValueAsDouble();
    inputs.flywheelVelocitys[0] = leftMotor.getVelocity().getValueAsDouble();
    inputs.flywheelVelocitys[1] = rightMotor.getVelocity().getValueAsDouble();
    inputs.flywheelVoltages[0] = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelVoltages[1] = rightMotor.getMotorVoltage().getValueAsDouble();
  }
  
}
