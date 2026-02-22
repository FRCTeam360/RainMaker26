// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class HopperRollerIOPB implements HopperRollerIO {
  private static final double GEAR_RATIO = 1.0; // FIXME: set actual gear ratio

  private final SparkFlex hopperRollerMotor =
      new SparkFlex(Constants.PracticeBotConstants.HOPPER_ROLLER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = hopperRollerMotor.getEncoder();
  private final SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();

  public HopperRollerIOPB() {
    sparkFlexConfig.idleMode(IdleMode.kBrake);
    sparkFlexConfig.inverted(true);

    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    hopperRollerMotor.configure(
        sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
