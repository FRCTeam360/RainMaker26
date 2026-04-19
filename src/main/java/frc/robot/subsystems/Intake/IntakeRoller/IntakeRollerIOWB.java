// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRoller;

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

public class IntakeRollerIOWB implements IntakeRollerIO {
  private final SparkFlex motor =
      new SparkFlex(WoodBotConstants.INTAKE_ROLLER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final DigitalInput sensor = new DigitalInput(WoodBotConstants.INTAKE_ROLLER_SENSOR_PORT);
  private final SparkClosedLoopController closedLoopConfig;

  public IntakeRollerIOWB() {

    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    config.smartCurrentLimit(55);

    config.closedLoop.p(0.0002).i(0.0).d(0.0);
    config.closedLoop.feedForward.kV(0.0018).kS(0.004);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopConfig = motor.getClosedLoopController();
  }

  @Override
  public void setDutyCycle(double duty) {
    motor.set(duty);
  }

  @Override
  public void stop() {
    this.setDutyCycle(0.0);
  }

  public void setEncoder(double value) {
    encoder.setPosition(value);
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoopConfig.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.p(kP).i(kI).d(kD);
    pidConfig.closedLoop.feedForward.kV(kV).kS(kS);
    motor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.position[0] = encoder.getPosition();
    inputs.sensor = sensor.get();
    inputs.supplyCurrent = 0;
    inputs.statorCurrent[0] = motor.getOutputCurrent(); // TODO: check i
    // this is right
    inputs.velocity[0] = encoder.getVelocity();
    inputs.voltage[0] = motor.getBusVoltage() * motor.getAppliedOutput();
  }
}
