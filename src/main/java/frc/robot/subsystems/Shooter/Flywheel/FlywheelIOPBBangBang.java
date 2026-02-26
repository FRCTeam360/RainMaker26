// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.PracticeBotConstants;

public class FlywheelIOPBBangBang implements FlywheelIO {

  private static final int BANG_BANG_SLOT = 0;
  private static final int PID_SLOT = 2;

  private static final double STATOR_CURRENT_LIMIT_AMPS = 200.0;
  private static final double MAX_NEGATIVE_TORQUE_CURRENT = 0.0;
  private static final double MAX_POSITIVE_TORQUE_CURRENT = 30.0;

  private static final double MAX_NEGATIVE_DUTY_CYCLE = 0.0;
  private static final double MAX_POSITIVE_DUTY_CYCLE = 1.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
  private static final double GEAR_RATIO = 1.0;

  private final TalonFX[] motors = {
    new TalonFX(PracticeBotConstants.FLYWHEEL_RIGHT_ID, PracticeBotConstants.CANBUS),
    new TalonFX(PracticeBotConstants.FLYWHEEL_LEFT_ID, PracticeBotConstants.CANBUS)
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

  public FlywheelIOPBBangBang() {
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

    // TODO TUNE IF NEEDED
    rightConfig.Slot2.kP = 3.0;
    rightConfig.Slot2.kI = 0.0;
    rightConfig.Slot2.kD = 0.1;
    rightConfig.Slot2.kA = 0.0;
    rightConfig.Slot2.kG = 0.0;
    rightConfig.Slot2.kS = 3.0;
    rightConfig.Slot2.kV = 0.008;

    rightConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Bang-bang control limits
    // Peak torque-current for ball/idle phases (constant torque)
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = MAX_POSITIVE_TORQUE_CURRENT;
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = MAX_NEGATIVE_TORQUE_CURRENT;

    // Peak duty-cycle for startup/recovery phases (max acceleration)
    rightConfig.MotorOutput.PeakForwardDutyCycle = MAX_POSITIVE_DUTY_CYCLE;
    rightConfig.MotorOutput.PeakReverseDutyCycle = MAX_NEGATIVE_DUTY_CYCLE;

    rightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    leftConfig = rightConfig.clone();
    // do not edit right configs after cloning
    leftConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    motors[1].getConfigurator().apply(leftConfig);
    motors[1].setNeutralMode(NeutralModeValue.Coast);
    motors[0].getConfigurator().apply(rightConfig);
    motors[0].setNeutralMode(NeutralModeValue.Coast);

    motors[1].setControl(
        new Follower(PracticeBotConstants.FLYWHEEL_RIGHT_ID, MotorAlignmentValue.Opposed));

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
        200,
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

  // Control request objects (reused for efficiency)
  private final VelocityDutyCycle dutyCycleBangBang =
      new VelocityDutyCycle(0.0).withSlot(BANG_BANG_SLOT);
  // Because this uses the VelocityTorqueCurrentFOC control mode, it will be limited by the torque
  // current config (30A limit from config)
  private final VelocityTorqueCurrentFOC torqueCurrentBangBang =
      new VelocityTorqueCurrentFOC(0.0).withSlot(BANG_BANG_SLOT);
  // PID uses VelocityVoltage instead - not limited by torque current config
  private final VelocityVoltage velocityPID = new VelocityVoltage(0.0).withSlot(PID_SLOT);

  @Override
  public void setSpinupVelocityControl(double velocityRPM) {
    // Startup/Recovery phase: Duty cycle bang-bang (max acceleration)
    motors[0].setControl(dutyCycleBangBang.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setHoldVelocityControl(double velocityRPM) {
    // Idle/Ball phase: Torque current bang-bang (consistent torque, 40A limit from config)
    motors[0].setControl(torqueCurrentBangBang.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setCoastVelocityControl(double velocityRPM) {
    // Traditional PID control using voltage control (not limited by torque current)
    // Limited only by supply current limit (100A) and voltage saturation
    motors[0].setControl(velocityPID.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

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
    inputs.velocities[0] = RotationsPerSecond.of(rightVelocitySignal.getValueAsDouble()).in(RPM);
    inputs.voltages[0] = rightMotorVoltageSignal.getValueAsDouble();

    inputs.statorCurrents[1] = leftStatorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrents[1] = leftSupplyCurrentSignal.getValueAsDouble();
    inputs.positions[1] = leftPositionSignal.getValueAsDouble();
    inputs.velocities[1] = RotationsPerSecond.of(leftVelocitySignal.getValueAsDouble()).in(RPM);
    inputs.voltages[1] = leftMotorVoltageSignal.getValueAsDouble();
  }
}
