// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TemporaryBotConstants;
import frc.robot.Constants.WoodBotConstants;

public class FlywheelIOProto implements FlywheelIO {

  private final TalonFX[] motors = {
    new TalonFX(TemporaryBotConstants.FLYWHEEL_RIGHT_ID, TemporaryBotConstants.CANBUS_NAME),
    new TalonFX(TemporaryBotConstants.FLYWHEEL_LEFT_ID, TemporaryBotConstants.CANBUS_NAME)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  public FlywheelIOProto() {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 0.0;
    double kV = 0.0;

    Slot0Configs slot0Configs = rightConfig.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    for (TalonFX i : motors) {
      i.getConfigurator().apply(defaultConfig);
    }

    rightConfig.CurrentLimits.StatorCurrentLimit = 160.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig
        .MotionMagic
        .withMotionMagicAcceleration(0.0)
        .withMotionMagicCruiseVelocity(0.0)
        .withMotionMagicJerk(0.0);
    rightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    leftConfig = rightConfig.clone();
    leftConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    boolean odd = false;
    for (TalonFX i : motors) {
      i.getConfigurator().apply(odd ? leftConfig : rightConfig);
      odd = !odd;
      i.setNeutralMode(NeutralModeValue.Coast);
    }

    boolean oddFollower = true;
    for (int i = 1; i < motors.length; i++) {
      motors[i].setControl(
          new Follower(
              WoodBotConstants.FLYWHEEL_RIGHT_ID,
              (oddFollower ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned)));
      oddFollower = !oddFollower;
    }
  }

  MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);

  @Override
  public void setRPM(double rpm) {
    double rps = rpm / 60.0;
    motors[0].setControl(velocityVoltage.withVelocity(rps));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.statorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.supplyCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.positions[i] = motors[i].getPosition().getValueAsDouble();
      inputs.velocities[i] = motors[i].getVelocity().getValueAsDouble()*60.0;
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }
}
