// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.WoodBotConstants;

public class FlywheelIOWB implements FlywheelIO {

  private final TalonFX[] motors = {
    new TalonFX(WoodBotConstants.FLYWHEEL_RIGHT_ID, WoodBotConstants.CANBUS),
    new TalonFX(WoodBotConstants.FLYWHEEL_LEFT_ID, WoodBotConstants.CANBUS)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  private final StatusSignal<Current> rightStatorCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;

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

    rightStatorCurrentSignal = motors[0].getStatorCurrent();
    rightSupplyCurrentSignal = motors[0].getSupplyCurrent();
    rightPositionSignal = motors[0].getPosition();
    rightVelocitySignal = motors[0].getVelocity();
    rightMotorVoltageSignal = motors[0].getMotorVoltage();

    leftStatorCurrentSignal = motors[1].getStatorCurrent();
    leftSupplyCurrentSignal = motors[1].getSupplyCurrent();
    leftPositionSignal = motors[1].getPosition();
    leftVelocitySignal = motors[1].getVelocity();
    leftMotorVoltageSignal = motors[1].getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        rightVelocitySignal,
        rightMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);
  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
  VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

  @Override
  public void setSpinupVelocityControl(double rpm) {
    double rps = rpm / 60.0;
    motors[0].setControl(velocityTorqueCurrent.withVelocity(rps));
  }

  @Override
  public void setHoldVelocityControl(double rpm) {
    double rps = rpm / 60.0;
    motors[0].setControl(velocityTorqueCurrent.withVelocity(rps));
  }

  @Override
  public void setCoastVelocityControl(double rpm) {
    double rps = rpm / 60.0;
    motors[0].setControl(velocityTorqueCurrent.withVelocity(rps));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        rightVelocitySignal,
        rightMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal);

    inputs.statorCurrents[0] = rightStatorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrents[0] = rightSupplyCurrentSignal.getValueAsDouble();
    inputs.positions[0] = rightPositionSignal.getValueAsDouble();
    // velocities are now in RPM
    inputs.velocities[0] = rightVelocitySignal.getValueAsDouble() * 60.0;
    inputs.voltages[0] = rightMotorVoltageSignal.getValueAsDouble();

    inputs.statorCurrents[1] = leftStatorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrents[1] = leftSupplyCurrentSignal.getValueAsDouble();
    inputs.positions[1] = leftPositionSignal.getValueAsDouble();
    // velocities are now in RPM
    inputs.velocities[1] = leftVelocitySignal.getValueAsDouble() * 60.0;
    inputs.voltages[1] = leftMotorVoltageSignal.getValueAsDouble();
  }
}
