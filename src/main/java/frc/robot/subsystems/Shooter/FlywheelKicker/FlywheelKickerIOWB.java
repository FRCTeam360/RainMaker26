// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.FlywheelKicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class FlywheelKickerIOWB implements FlywheelKickerIO {
  /** Creates a new FlywheelKickerIOWB. */
  private final SparkMax flywheelKickerMotor =
      new SparkMax(Constants.WoodBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = flywheelKickerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController closedLoopController;

  private final CANrange canSensor =
      new CANrange(Constants.WoodBotConstants.FLYWHEEL_KICKER_SENSOR_ID, Constants.RIO_CANBUS);

  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> isDetectedSignal;

  public FlywheelKickerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    sparkMaxConfig.smartCurrentLimit(40);

    sparkMaxConfig.closedLoop.p(0.0002).i(0.0).d(0.0);
    sparkMaxConfig.closedLoop.feedForward.kV(0.0021).kS(0.04);

    flywheelKickerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = flywheelKickerMotor.getClosedLoopController();

    CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
    sensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // unknown unit
    sensorConfig.ProximityParams.ProximityThreshold = 0.1; // meters
    sensorConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRangeUserFreq);
    canSensor.getConfigurator().apply(sensorConfig);

    distanceSignal = canSensor.getDistance();
    isDetectedSignal = canSensor.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(50, distanceSignal, isDetectedSignal);
    canSensor.optimizeBusUtilization();
  }

  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelKickerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        flywheelKickerMotor.getOutputCurrent()
            * flywheelKickerMotor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelKickerMotor.getBusVoltage() * flywheelKickerMotor.getAppliedOutput();
    BaseStatusSignal.refreshAll(distanceSignal, isDetectedSignal);
    inputs.sensorProximity = distanceSignal.getValueAsDouble();
    inputs.sensorActivated = isDetectedSignal.getValue();
  }

  public void setDutyCycle(double dutyCycle) {
    flywheelKickerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
