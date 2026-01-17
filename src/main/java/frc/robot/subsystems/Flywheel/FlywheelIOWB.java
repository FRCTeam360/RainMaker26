// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants.WoodBotConstants;

public class FlywheelIOWB implements FlywheelIO {

  private final TalonFX[] motors = { new TalonFX(WoodBotConstants.FLYWHEEL0_ID, WoodBotConstants.CANBUS_NAME),
      new TalonFX(WoodBotConstants.FLYWHEEL1_ID, WoodBotConstants.CANBUS_NAME) };
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  public FlywheelIOWB() {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 0.0;
    double kV = 0.0;
    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    config.MotionMagic
        .withMotionMagicAcceleration(0.0)
        .withMotionMagicCruiseVelocity(0.0)
        .withMotionMagicJerk(0.0);
    config.MotorOutput = motorOutputConfigs;

    for (int i = 1; i < motors.length; i++) {
      motors[i].setControl(new Follower(WoodBotConstants.FLYWHEEL0_ID, MotorAlignmentValue.Aligned));
    }
    motors[0].getConfigurator().apply(config);

    // config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
  }

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

  @Override
  public void setRPM(double rpm) {
    double rps = rpm/60.0;
    motors[0].setControl(velocityDutyCycle.withVelocity(rps));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.statorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.positions[i] = motors[i].getPosition().getValueAsDouble();
      inputs.velocities[i] = motors[i].getVelocity().getValueAsDouble();
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }

}
