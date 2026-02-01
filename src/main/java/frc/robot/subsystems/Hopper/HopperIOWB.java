// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.WoodBotConstants;

public class HopperIOWB implements HopperIO {
  /** Creates a new HopperIOWB. */
  private final SparkMax motor =
      new SparkMax(Constants.WoodBotConstants.HOPPER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.HOPPER_SENSOR_ID);

  public HopperIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    // sparkMaxConfig.smartCurrentLimit(40);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(HopperIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = motor.getOutputCurrent() * motor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.sensor = sensor.get();
  }

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }
}
