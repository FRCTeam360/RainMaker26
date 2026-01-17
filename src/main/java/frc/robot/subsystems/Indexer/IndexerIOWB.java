// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerIOWB implements IndexerIO {
  /** Creates a new IndexerIOWB. */

  private final SparkMax indexerMotor = new SparkMax(Constants.WoodBotConstants.INDEXER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  private final DigitalInput sensor = new DigitalInput(Constants.INDEXER_SENSOR_PORT);

  public IndexerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);

    indexerMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerPosition = encoder.getPosition();
    inputs.indexerStatorCurrent = indexerMotor.getOutputCurrent();
    inputs.indexerSupplyCurrent = indexerMotor.getOutputCurrent() * indexerMotor.getAppliedOutput(); // TODO: check if this is right
    inputs.indexerVelocity = encoder.getVelocity();
    inputs.indexerVoltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.indexerSensor = sensor.get();
  }
  
  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }
}
