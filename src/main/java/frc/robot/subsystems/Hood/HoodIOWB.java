// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodIOWB implements HoodIO {
  // /** Creates a new HoodIOWB. */
  private final SparkMax hoodMotor = new SparkMax(Constants.WoodBotConstants.HOOD_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = hoodMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  SparkClosedLoopController controller = hoodMotor.getClosedLoopController();

  public void setEncoder(double position) {
    encoder.setPosition(position);
  }

  public HoodIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    // CAD doesn't know what motor type it is, we set to assume sparkmax.

    hoodMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkMaxConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0);
  }

  public void setPosition(double position) {
    // encoder.setPosition(position);
    controller.setSetpoint(position, ControlType.kPosition);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = hoodMotor.getOutputCurrent();
    inputs.supplyCurrent = hoodMotor.getOutputCurrent() * hoodMotor.getAppliedOutput(); // TODO: check if this is
                                                                                        // right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = hoodMotor.getBusVoltage() * hoodMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }

}
