// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class FlywheelIOWB implements FlywheelIO {

  // need motor vvvvvv
  private final TalonFX leftMotor = new TalonFX(0);//need valid id for motor
  private final TalonFX rightMotor = new TalonFX(0);//need valid id for motor
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

  public FlywheelIOWB(){

  }

  public void setDutyCycle(double dutyCycle) {
    leftMotor.set(dutyCycle);
    rightMotor.set(dutyCycle);
  }

  @Override
  public void setRPM(double rpm, ControlType kvelocity) 
    final double kA = 0.0;
    final double kD = 0.0;
    final double kG = 0.0;
    final double kI = 0.0;
    final double kP = 0.0;
    final double kS = 0.0;

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

    leftMotor.getConfigurator().apply(config, 0.0);
    rightMotor.getConfigurator().apply(config, 0.0);
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public double getPower() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPower'");
  }

  @Override
  public double getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }
  
}
