// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FlywheelKicker;

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

public class FlywheelKickerIOWB implements FlywheelKickerIO {
  /** Creates a new FlywheelKickerIOWB. */
  private final SparkMax flywheelkickerMotor =
      new SparkMax(Constants.WoodBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = flywheelkickerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.FLYWHEEL_KICKER_SENSOR_ID);

  public FlywheelKickerIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    sparkMaxConfig.smartCurrentLimit(40);

    flywheelkickerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelkickerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        flywheelkickerMotor.getOutputCurrent()
            * flywheelkickerMotor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelkickerMotor.getBusVoltage() * flywheelkickerMotor.getAppliedOutput();
    inputs.sensor = sensor.get();
  }

  public void setDutyCycle(double dutyCycle) {
    flywheelkickerMotor.set(dutyCycle);
  }
}
