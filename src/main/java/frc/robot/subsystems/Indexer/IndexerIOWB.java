// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

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

public class IndexerIOWB implements IndexerIO {
  /** Creates a new IndexerIOWB. */
  private final SparkMax indexerMotor =
      new SparkMax(Constants.WoodBotConstants.INDEXER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController closedLoopController;

  private final double CONVERSION_FACTOR = 1.0;

  public IndexerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    sparkMaxConfig.smartCurrentLimit(40);
    sparkMaxConfig
        .analogSensor
        .positionConversionFactor(CONVERSION_FACTOR)
        .velocityConversionFactor(CONVERSION_FACTOR);
    sparkMaxConfig.closedLoop.p(0.0002).i(0.0).d(0.0);
    sparkMaxConfig.closedLoop.feedForward.kV(0.0021).kS(0.04);

    indexerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = indexerMotor.getClosedLoopController();
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

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
