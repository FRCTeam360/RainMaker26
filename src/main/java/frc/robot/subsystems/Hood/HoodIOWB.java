// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodIOWB implements HoodIO {
  /** Creates a new HoodIOWB. */

  private final SparkMax hoodMotor = new SparkMax(Constants.WoodBotConstants.HOOD_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = hoodMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public HoodIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    // CAD doesn't know what motor type it is, we set to assume sparkmax.

    hoodMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodPosition = encoder.getPosition();
    inputs.hoodStatorCurrent = hoodMotor.getOutputCurrent();
    inputs.hoodSupplyCurrent = hoodMotor.getOutputCurrent() * hoodMotor.getAppliedOutput(); // TODO: check if this is
                                                                                            // right
    inputs.hoodVelocity = encoder.getVelocity();
    inputs.hoodVoltage = hoodMotor.getBusVoltage() * hoodMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
