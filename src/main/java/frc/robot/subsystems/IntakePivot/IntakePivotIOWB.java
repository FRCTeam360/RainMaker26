// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakePivot.IntakePivotIO.IntakePivotIOInputs;

public class IntakePivotIOWB extends SubsystemBase {
  private final TalonFX intakePivot = new TalonFX(1, "intakePivot");
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  /** Creates a new IntakePivotIOWB. */
  public IntakePivotIOWB() {
    
  }

  public void setPosition(double value) {
    intakePivot.setPosition(value);
  }

  public void stop() {
    intakePivot.stopMotor();
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = intakePivot.getPosition().getValueAsDouble();
    inputs.statorCurrent = intakePivot.getStatorCurrent().getValueAsDouble();
    inputs.velocity = intakePivot.getVelocity().getValueAsDouble();
    inputs.voltage = intakePivot.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = intakePivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
