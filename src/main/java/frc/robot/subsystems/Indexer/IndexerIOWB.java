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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerIOWB implements IndexerIO {
  /** Creates a new IndexerIOWB. */

  private final SparkMax indexerMotor = new SparkMax(Constants.WoodBotConstants.INDEXER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public IndexerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);

    indexerMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerPosition = encoder.getPosition();
    inputs.indexerStatorCurrent = indexerMotor.getOutputCurrent();
    inputs.indexerSupplyCurrent = indexerMotor.get;
    inputs.indexerVelocity;
    inputs.indexerVoltage;
  }
  
  public void setDutyCycle(double dutyCycle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDutyCycle'");
  }
}
