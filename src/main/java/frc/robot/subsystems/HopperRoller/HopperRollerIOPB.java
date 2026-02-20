// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class HopperRollerIOPB implements HopperRollerIO {

  private final SparkMax hopperRollerMotor =
      new SparkMax(Constants.PracticeBotConstants.HOPPER_ROLLER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = hopperRollerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public HopperRollerIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);

    hopperRollerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(HopperRollerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = hopperRollerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        hopperRollerMotor.getOutputCurrent() * hopperRollerMotor.getAppliedOutput();

    inputs.velocity = encoder.getVelocity();
    inputs.voltage = hopperRollerMotor.getBusVoltage() * hopperRollerMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    hopperRollerMotor.set(dutyCycle);
  }
}
