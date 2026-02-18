// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
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
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkClosedLoopController controller;
  private static final double CONVERSION_FACTOR = 1.0;

  public void setEncoder(double position) {
    encoder.setPosition(position);
  }

  public HoodIOWB() {

    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config
        .analogSensor
        .positionConversionFactor(CONVERSION_FACTOR)
        .velocityConversionFactor(CONVERSION_FACTOR);

    // Smart current limit
    config.smartCurrentLimit(40);

    // PID gains
    config.closedLoop.p(0.1).i(0.0).d(0.0);

    // Soft limits
    config
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(10.0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0.0);

    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
