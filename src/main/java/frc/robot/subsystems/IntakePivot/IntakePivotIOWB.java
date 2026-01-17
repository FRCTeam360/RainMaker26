// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot.IntakePivotIO.IntakePivotIOInputs;

public class IntakePivotIOWB implements IntakePivotIO {
  private final TalonFX intakePivot = new TalonFX(Constants.WoodBotConstants.INTAKE_PIVOT_PORT, "intakePivot");
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

  /** Creates a new IntakePivotIOWB. */
  public IntakePivotIOWB() {
    intakePivot.getConfigurator().apply(config);
    intakePivot.setNeutralMode(NeutralModeValue.Brake);
    
  }

  public void setPosition(double value) {
    intakePivot.setPosition(value);
  }

  public void setDutyCycle(double value) {
    intakePivot.set(value);
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = intakePivot.getPosition().getValueAsDouble();
    inputs.statorCurrent = intakePivot.getStatorCurrent().getValueAsDouble();
    inputs.velocity = intakePivot.getVelocity().getValueAsDouble();
    inputs.voltage = intakePivot.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = intakePivot.getSupplyCurrent().getValueAsDouble();
  }

}
