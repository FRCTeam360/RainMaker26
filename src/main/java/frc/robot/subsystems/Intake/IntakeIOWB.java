// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.WoodBotConstants;

public class IntakeIOWB implements IntakeIO {
  private final SparkFlex motor = new SparkFlex(WoodBotConstants.INTAKE_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.INDEXER_SENSOR_PORT);
  private final SparkClosedLoopController closedLoopConfig;

  public IntakeIOWB() {

    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    config.smartCurrentLimit(40);

    config.closedLoop.p(0.0002).i(0.0).d(0.0);
    config.closedLoop.feedForward.kV(0.0018).kS(0.004);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopConfig = motor.getClosedLoopController();
  }

  public void setDutyCycle(double duty) {
    motor.set(duty);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  public void setEncoder(double value) {
    encoder.setPosition(value);
  }

  public void setVelocity(double velocity) {
    closedLoopConfig.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.sensor = sensor.get();
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = motor.getOutputCurrent() * motor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
  }
}
