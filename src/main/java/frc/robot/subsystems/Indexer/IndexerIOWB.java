// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
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

public class IndexerIOWB implements IndexerIO {
  /** Creates a new IndexerIOWB. */
  CANrangeConfiguration intakeConfig = new CANrangeConfiguration();

  private final SparkMax indexerMotor =
      new SparkMax(Constants.WoodBotConstants.INDEXER_ID, MotorType.kBrushless);
  private final CANrange intakeSensor =
      new CANrange(
          Constants.WoodBotConstants.INDEXER_SENSOR_ID, Constants.WoodBotConstants.CANBUS_NAME);
  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.INDEXER_SENSOR_ID);

  public IndexerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    // sparkMaxConfig.smartCurrentLimit(40);

    indexerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CANrangeConfiguration intakeConfig = new CANrangeConfiguration();
    intakeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    intakeConfig.ProximityParams.ProximityThreshold = 0.1;
    intakeConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRangeUserFreq);
    intakeSensor.getConfigurator().apply(intakeConfig);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = indexerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        indexerMotor.getOutputCurrent() * indexerMotor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    // inputs.intakeSensorProximity = intakeSensor.getDistance().refresh().getValueAsDouble();
    // inputs.fuelDetected = intakeSensor.getIsDetected().getValue();
  }

  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }

  public void refreshData() {
    BaseStatusSignal.refreshAll(intakeSensor.getIsDetected(true));
  }
}
