// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.WoodBotConstants;

public class IndexerIOWB implements IndexerIO {
  /** Creates a new IndexerIOWB. */
  private final SparkMax indexerMotor =
      new SparkMax(Constants.WoodBotConstants.INDEXER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();

  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.INDEXER_ID);

  private final double CONVERSION_FACTOR = 1.0;

  public IndexerIOWB() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    config.smartCurrentLimit(40);
    config
        .analogSensor
        .positionConversionFactor(CONVERSION_FACTOR)
        .velocityConversionFactor(CONVERSION_FACTOR);

    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = indexerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        indexerMotor.getOutputCurrent() * indexerMotor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }
}
