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
import frc.robot.Constants.PracticeBotConstants;

public class IntakeRollerIOPB implements IntakeRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int STALL_CURRENT_LIMIT_AMPS = 50;
  private static final int FREE_CURRENT_LIMIT_AMPS = 50;

  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV = 0.0019;
  private static final double KS = 0.04;

  private final SparkFlex motor =
      new SparkFlex(PracticeBotConstants.INTAKE_ROLLER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  public IntakeRollerIOPB() {
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);
    config.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);

    config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    config.closedLoop.p(KP).i(KI).d(KD);
    config.closedLoop.feedForward.kV(KV).kS(KS);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = motor.getClosedLoopController();
  }

  public void setDutyCycle(double duty) {
    motor.set(duty);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  public void setVelocity(double velocity) {
    closedLoopController.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = motor.getOutputCurrent();
    inputs.supplyCurrent = 0;
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
  }
}
