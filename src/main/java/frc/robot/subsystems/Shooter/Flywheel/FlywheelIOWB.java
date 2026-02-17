// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.WoodBotConstants;

public class FlywheelIOWB implements FlywheelIO {

  private final TalonFX[] motors = {
      new TalonFX(WoodBotConstants.FLYWHEEL_RIGHT_ID, WoodBotConstants.CANBUS_NAME),
      new TalonFX(WoodBotConstants.FLYWHEEL_LEFT_ID, WoodBotConstants.CANBUS_NAME)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  public FlywheelIOWB() {
    // Slot 0: Bang-bang control configuration
    // Very high kP creates bang-bang behavior (either full output or zero)
    rightConfig.Slot0.kP = 999999.0;
    rightConfig.Slot0.kI = 0.0;
    rightConfig.Slot0.kD = 0.0;
    rightConfig.Slot0.kA = 0.0;
    rightConfig.Slot0.kG = 0.0;
    rightConfig.Slot0.kS = 0.0;
    rightConfig.Slot0.kV = 0.0;

    // Slot 2: Traditional PID control configuration
    // Standard velocity PID gains
    rightConfig.Slot2.kP = 3.0;
    rightConfig.Slot2.kI = 0.0;
    rightConfig.Slot2.kD = 0.1;
    rightConfig.Slot2.kA = 0.0;
    rightConfig.Slot2.kG = 0.0;
    rightConfig.Slot2.kS = 3.0;
    rightConfig.Slot2.kV = 0.008;

    rightConfig.CurrentLimits.StatorCurrentLimit = 200.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Bang-bang control limits
    // Peak torque-current for ball/idle phases (constant torque)
    // Initial value - will be updated via LoggedTunableNumber
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    
    // Peak duty-cycle for startup/recovery phases (max acceleration)
    rightConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    rightConfig.MotorOutput.PeakReverseDutyCycle = 0.0;


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

  // Control request objects (reused for efficiency)
  private final VelocityDutyCycle dutyCycleBangBang = new VelocityDutyCycle(0.0).withSlot(0);
  private final VelocityTorqueCurrentFOC torqueCurrentBangBang = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  // PID uses VelocityVoltage instead - not limited by torque current config
  private final VelocityVoltage velocityPID = new VelocityVoltage(0.0).withSlot(2);

  @Override
  public void setVelocityBangBang(double velocityRPS) {
    // Startup/Recovery phase: Duty cycle bang-bang (max acceleration)
    motors[0].setControl(dutyCycleBangBang.withVelocity(velocityRPS));
  }

  @Override
  public void setVelocityTorqueCurrentBangBang(double velocityRPS) {
    // Idle/Ball phase: Torque current bang-bang (consistent torque, 40A limit from config)
    motors[0].setControl(torqueCurrentBangBang.withVelocity(velocityRPS));
  }

  @Override
  public void setVelocityPID(double velocityRPS) {
    // Traditional PID control using voltage control (not limited by torque current)
    // Limited only by supply current limit (100A) and voltage saturation
    motors[0].setControl(velocityPID.withVelocity(velocityRPS));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  @Override
  public void stop() {
    motors[0].set(0.0);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.statorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.supplyCurrents[i] = motors[i].getSupplyCurrent().getValueAsDouble();
      inputs.positions[i] = motors[i].getPosition().getValueAsDouble();
      inputs.velocities[i] = motors[i].getVelocity().getValueAsDouble();
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }
}
