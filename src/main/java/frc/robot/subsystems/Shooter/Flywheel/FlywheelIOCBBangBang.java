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
import frc.robot.Constants;

/**
 * Comp bot flywheel IO implementation using bang-bang control on dual TalonFX motors.
 *
 * <p>This implementation provides three control modes via separate CTRE control request slots:
 *
 * <ul>
 *   <li><b>Spinup (Slot 0, duty-cycle bang-bang)</b> — extremely high kP drives full duty cycle
 *       when below setpoint, zero when above. Unconstrained by torque limits for maximum
 *       acceleration during initial spinup and recovery after shots.
 *   <li><b>Hold (Slot 0, torque-current bang-bang)</b> — same bang-bang gains but applied through
 *       FOC torque-current control, clamped to {@value #MAX_POSITIVE_TORQUE_CURRENT}A. Provides
 *       consistent, bounded torque for holding velocity at setpoint.
 *   <li><b>Coast (Slot 2, PID voltage)</b> — traditional velocity PID for smooth control when
 *       precision matters more than response time.
 * </ul>
 *
 * <p>The right motor is the leader; the left motor follows in opposed direction.
 */
public class FlywheelIOCBBangBang implements FlywheelIO {

  /** CTRE slot index for bang-bang control (extremely high kP). */
  private static final int BANG_BANG_SLOT = 0;

  /** CTRE slot index for traditional PID velocity control. */
  private static final int PID_SLOT = 2;

  private static final double STATOR_CURRENT_LIMIT_AMPS = 200.0;
  private static final double MAX_NEGATIVE_TORQUE_CURRENT = 0.0;
  private static final double MAX_POSITIVE_TORQUE_CURRENT = 40.0;

  private static final double MAX_NEGATIVE_DUTY_CYCLE = 0.0;
  private static final double MAX_POSITIVE_DUTY_CYCLE = 1.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
  private static final double GEAR_RATIO = 1.0;
  private final TalonFX[] motors;
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  private final StatusSignal<Current> rightStatorCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightTorqueCurrentSignal;
  private final StatusSignal<Double> rightDutyCycleSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;

  /** Frequency for telemetry signals (current, position, supply current). */
  private static final double TELEMETRY_UPDATE_FREQUENCY_HZ = 50.0;

  /** Frequency for control-critical signals (velocity, voltage, torque current, duty cycle). */
  private static final double CONTROL_UPDATE_FREQUENCY_HZ = 500.0;

  /** Constructs the comp bot flywheel IO and configures both TalonFX motors. */
  public FlywheelIOCBBangBang() {
    motors =
        new TalonFX[] {
          new TalonFX(
              Constants.CompBotConstants.FLYWHEEL_RIGHT_ID, Constants.CompBotConstants.CANBUS),
          new TalonFX(
              Constants.CompBotConstants.FLYWHEEL_LEFT_ID, Constants.CompBotConstants.CANBUS)
        };

    // Reset all motors to factory defaults before applying custom config
    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    for (TalonFX motor : motors) {
      motor.getConfigurator().apply(defaultConfig);
    }

    // Slot 0: Bang-bang — extremely high kP drives full output below setpoint, zero above
    rightConfig.Slot0.kP = 999999.0;
    rightConfig.Slot0.kI = 0.0;
    rightConfig.Slot0.kD = 0.0;
    rightConfig.Slot0.kA = 0.0;
    rightConfig.Slot0.kG = 0.0;
    rightConfig.Slot0.kS = 0.0;
    rightConfig.Slot0.kV = 0.0;

    // Slot 2: Traditional PID velocity control
    // TODO: Tune PID gains if needed
    rightConfig.Slot2.kP = 0.06;
    rightConfig.Slot2.kI = 0.0;
    rightConfig.Slot2.kD = 0.0;
    rightConfig.Slot2.kA = 0.0;
    rightConfig.Slot2.kG = 0.0;
    rightConfig.Slot2.kS = 0.33;
    rightConfig.Slot2.kV = 0.125;

    rightConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Torque-current limits for hold phase (bounded torque for consistent hold)
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = MAX_POSITIVE_TORQUE_CURRENT;
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = MAX_NEGATIVE_TORQUE_CURRENT;

    // Duty-cycle limits for spinup/recovery phase (unconstrained acceleration)
    rightConfig.MotorOutput.PeakForwardDutyCycle = MAX_POSITIVE_DUTY_CYCLE;
    rightConfig.MotorOutput.PeakReverseDutyCycle = MAX_NEGATIVE_DUTY_CYCLE;

    rightConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    leftConfig = rightConfig.clone();
    // Do not edit rightConfig after cloning — leftConfig is an independent copy
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    motors[1].getConfigurator().apply(leftConfig);
    motors[1].setNeutralMode(NeutralModeValue.Coast);
    motors[0].getConfigurator().apply(rightConfig);
    motors[0].setNeutralMode(NeutralModeValue.Coast);

    motors[1].setControl(
        new Follower(Constants.CompBotConstants.FLYWHEEL_RIGHT_ID, MotorAlignmentValue.Opposed));

    rightStatorCurrentSignal = motors[0].getStatorCurrent();
    rightSupplyCurrentSignal = motors[0].getSupplyCurrent();
    rightPositionSignal = motors[0].getPosition();
    rightVelocitySignal = motors[0].getVelocity();
    rightMotorVoltageSignal = motors[0].getMotorVoltage();
    rightTorqueCurrentSignal = motors[0].getTorqueCurrent();
    rightDutyCycleSignal = motors[0].getDutyCycle();

    leftStatorCurrentSignal = motors[1].getStatorCurrent();
    leftSupplyCurrentSignal = motors[1].getSupplyCurrent();
    leftPositionSignal = motors[1].getPosition();
    leftVelocitySignal = motors[1].getVelocity();
    leftMotorVoltageSignal = motors[1].getMotorVoltage();

    // Leader signals required by the follower at 500 Hz (per CTRE docs:
    // DutyCycle, MotorVoltage, and TorqueCurrent must remain enabled on the leader).
    BaseStatusSignal.setUpdateFrequencyForAll(
        CONTROL_UPDATE_FREQUENCY_HZ,
        rightMotorVoltageSignal,
        rightTorqueCurrentSignal,
        rightDutyCycleSignal);

    // Telemetry signals at 200 Hz — all other leader and follower signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        TELEMETRY_UPDATE_FREQUENCY_HZ,
        rightVelocitySignal,
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  /** Duty-cycle bang-bang request for spinup/recovery (Slot 0, unconstrained acceleration). */
  private final VelocityDutyCycle dutyCycleBangBang =
      new VelocityDutyCycle(0.0).withSlot(BANG_BANG_SLOT);

  /**
   * Torque-current bang-bang request for holding at setpoint (Slot 0, capped at {@value
   * #MAX_POSITIVE_TORQUE_CURRENT}A by config).
   */
  private final VelocityTorqueCurrentFOC torqueCurrentBangBang =
      new VelocityTorqueCurrentFOC(0.0).withSlot(BANG_BANG_SLOT);

  /** Traditional PID velocity request for smooth coast control (Slot 2, voltage-based). */
  private final VelocityVoltage velocityPID = new VelocityVoltage(0.0).withSlot(PID_SLOT);

  /** {@inheritDoc} */
  @Override
  public void setSpinupVelocityControl(double velocityRPM) {
    motors[0].setControl(dutyCycleBangBang.withVelocity(velocityRPM / 60.0));
  }

  /** {@inheritDoc} */
  @Override
  public void setHoldVelocityControl(double velocityRPM) {
    motors[0].setControl(torqueCurrentBangBang.withVelocity(velocityRPM / 60.0));
  }

  /** {@inheritDoc} */
  @Override
  public void setCoastVelocityControl(double velocityRPM) {
    motors[0].setControl(velocityPID.withVelocity(velocityRPM / 60.0));
  }

  /** {@inheritDoc} */
  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  /** {@inheritDoc} */
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
