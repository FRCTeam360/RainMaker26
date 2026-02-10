// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WoodBotConstants;

public class FlywheelIOWB implements FlywheelIO {

  private final TalonFX[] motors = {
    new TalonFX(WoodBotConstants.FLYWHEEL_RIGHT_ID, WoodBotConstants.CANBUS_NAME),
    new TalonFX(WoodBotConstants.FLYWHEEL_LEFT_ID, WoodBotConstants.CANBUS_NAME)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kP", 2.0);
  private final LoggedNetworkNumber tunableKi =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kI", 0.0);
  private final LoggedNetworkNumber tunableKd =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kD", 0.1);
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/Flywheel/SetpointRPM", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/FlywheelKicker/Enabled", false);


  public FlywheelIOWB() {
    double kP = 3.0;
    double kI = 0.0;
    double kD = 0.1;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 3.0;
    double kV = 0.008;

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

    rightConfig.CurrentLimits.StatorCurrentLimit = 200.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
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
  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
  VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

    public void updateTunable() {
      // needs testing
    if (tuningEnabled.get()) {
      Slot0Configs slot0 = new Slot0Configs();
      motors[0].getConfigurator().refresh(slot0);
      slot0.kP = tunableKp.get();
      slot0.kI = tunableKi.get();
      slot0.kD = tunableKd.get();
      setVelocityTunable(() -> tunableSetpoint.get());
      motors[0].getConfigurator().apply(slot0);
    }
  }


  @Override
  public void setVelocity(double rpm) {
    double rps = rpm / 60.0;
    motors[0].setControl(velocityTorqueCurrent.withVelocity(rps));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  public void setVelocityTunable(DoubleSupplier doubleSupplier) {
    this.setVelocity(tunableSetpoint.getAsDouble());
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.statorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.supplyCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.positions[i] = motors[i].getPosition().getValueAsDouble();
      // velocities are now in RPM
      inputs.velocities[i] = motors[i].getVelocity().getValueAsDouble() * 60.0;
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }
}
