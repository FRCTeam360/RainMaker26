// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOWB extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(0, "default name");
  private final TalonFXConfiguration talonFXconfigs = new TalonFXConfiguration();
  private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  /** Creates a new IntakeIOWB. */
  public IntakeIOWB() {

  }

  public void setDutyCycle(double duty) {
    intakeMotor.set(duty);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public void setEncoder(double value) {
    intakeMotor.setPosition(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
