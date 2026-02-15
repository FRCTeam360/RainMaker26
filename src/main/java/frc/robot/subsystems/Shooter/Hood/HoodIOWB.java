// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class HoodIOWB implements HoodIO {
  // /** Creates a new HoodIOWB. */
  private final SparkMax hoodMotor =
      new SparkMax(Constants.WoodBotConstants.HOOD_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = hoodMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController controller;

  public void setEncoder(double position) {
    encoder.setPosition(position);
  }

  public HoodIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);

    // Smart current limit
    sparkMaxConfig.smartCurrentLimit(40);

    // PID gains
    sparkMaxConfig.closedLoop.p(0.21).i(0.0).d(0.0);

    // Soft limits
    sparkMaxConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(24.0);

    hoodMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = hoodMotor.getClosedLoopController();
  }

  public void setPosition(double position) {
    // old:encoder.setPosition(position);
    controller.setSetpoint(position, ControlType.kPosition);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = hoodMotor.getOutputCurrent() * hoodMotor.getAppliedOutput();
    inputs.supplyCurrent = hoodMotor.getOutputCurrent();
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = hoodMotor.getBusVoltage() * hoodMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
