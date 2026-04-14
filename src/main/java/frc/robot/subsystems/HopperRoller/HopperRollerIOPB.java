// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class HopperRollerIOPB implements HopperRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final int STALL_CURRENT_LIMIT_AMPS = 70;
  private static final int FREE_CURRENT_LIMIT_AMPS = 40;

  private static final double KP = 0.000375; // can be 0.000350
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV = 0.0017;
  private static final double KS = 0.04;

  private final SparkFlex hopperRollerMotor =
      new SparkFlex(Constants.PracticeBotConstants.HOPPER_ROLLER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = hopperRollerMotor.getEncoder();
  private final SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  public HopperRollerIOPB() {
    sparkFlexConfig.idleMode(IdleMode.kBrake);
    sparkFlexConfig.inverted(true);
    sparkFlexConfig.smartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_CURRENT_LIMIT_AMPS);

    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.quadratureMeasurementPeriod(25);
    sparkFlexConfig.encoder.quadratureAverageDepth(8);

    sparkFlexConfig.closedLoop.p(KP).i(KI).d(KD);
    sparkFlexConfig.closedLoop.feedForward.kV(KV).kS(KS);

    hopperRollerMotor.configure(
        sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = hopperRollerMotor.getClosedLoopController();
  }

  public void updateInputs(HopperRollerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = hopperRollerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        hopperRollerMotor.getOutputCurrent() * hopperRollerMotor.getAppliedOutput();
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = hopperRollerMotor.getBusVoltage() * hopperRollerMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    hopperRollerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
