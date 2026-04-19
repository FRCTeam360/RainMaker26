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
import frc.robot.Constants.CompBotConstants;

public class IntakeRollerIOCB implements IntakeRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int STALL_CURRENT_LIMIT_AMPS = 75;
  private static final int FREE_CURRENT_LIMIT_AMPS = 60;

  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV = 0.0017;
  private static final double KS = 0.01;

  private final SparkFlex motorLeft;
  private final SparkFlex motorRight;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkFlexConfig leftConfig = new SparkFlexConfig();
  private final SparkFlexConfig rightConfig = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  public IntakeRollerIOCB() {
    motorLeft = new SparkFlex(CompBotConstants.LEFT_INTAKE_ROLLER_ID, MotorType.kBrushless);
    motorRight = new SparkFlex(CompBotConstants.RIGHT_INTAKE_ROLLER_ID, MotorType.kBrushless);
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();

    leftConfig.idleMode(IdleMode.kCoast);
    rightConfig.idleMode(IdleMode.kCoast);

    leftConfig.inverted(true);

    leftConfig.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);
    rightConfig.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);

    leftConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    leftConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);
    leftConfig.encoder.quadratureMeasurementPeriod(25);
    leftConfig.encoder.quadratureAverageDepth(8);

    leftConfig.closedLoop.p(KP).i(KI).d(KD);
    leftConfig.closedLoop.feedForward.kV(KV).kS(KS);

    motorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightConfig.follow(motorLeft, true);
    motorRight.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motorLeft.getClosedLoopController();
  }

  @Override
  public void setDutyCycle(double duty) {
    motorLeft.set(duty);
  }

  @Override
  public void stop() {
    this.setDutyCycle(0.0);
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoopController.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.p(kP).i(kI).d(kD);
    pidConfig.closedLoop.feedForward.kV(kV).kS(kS);
    motorLeft.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.position[0] = leftEncoder.getPosition();
    inputs.statorCurrent[0] = motorLeft.getOutputCurrent();
    inputs.velocity[0] = leftEncoder.getVelocity();
    inputs.voltage[0] = motorLeft.getBusVoltage() * motorLeft.getAppliedOutput();
    inputs.position[1] = rightEncoder.getPosition();
    inputs.statorCurrent[1] = motorRight.getOutputCurrent();
    inputs.velocity[1] = rightEncoder.getVelocity();
    inputs.voltage[1] = motorRight.getBusVoltage() * motorRight.getAppliedOutput();
    inputs.supplyCurrent = 0;
  }
}
