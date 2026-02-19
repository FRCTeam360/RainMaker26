// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class HopperIOWB implements HopperIO {
  /** Creates a new HopperIOWB. */
  private final SparkMax motor =
      new SparkMax(Constants.WoodBotConstants.HOPPER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public HopperIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(HopperIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = motor.getOutputCurrent() * motor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }
}
