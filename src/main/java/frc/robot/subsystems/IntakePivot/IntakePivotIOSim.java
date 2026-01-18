// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class IntakePivotIOSim implements IntakePivotIO {

  // Motor constants
  private double gearRatio = 10.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final double kP = 7.0;
  private final double kI = 0.0;
  private final double kD = 0.5;
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  private final double kG = 0.75;  // Gravity compensation
  private final double armLength = 0.762;  // meters (30 inches)
  private final double armMass = 2.0;  // kg

  // Motor and control
  private final TalonFX motor = new TalonFX(22);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Simulation
  private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
      gearbox,
      gearRatio,
      SingleJointedArmSim.estimateMOI(armLength, armMass),
      armLength,
      Units.degreesToRadians(-75),
      Units.degreesToRadians(255),
      true,  // Simulate gravity
      0);

  public IntakePivotIOSim() {
    // Configure TalonFX with PID and gravity compensation
    configureMotor();
    
    // Initialize motor sim state to match arm sim initial position 
    // // full disclosure this was an AI troubleshooting addition and I don't know it's necessary.

    motor.getSimState().setRawRotorPosition(
        Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
    motor.getSimState().setRotorVelocity(
        RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec() * gearRatio).in(RotationsPerSecond));
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Configure PID gains for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;  // Gravity compensation
    
    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = 180.0;
    currentLimits.StatorCurrentLimitEnable = true;
    
    // Set brake mode (hold position when no command)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    config.Feedback.SensorToMechanismRatio = gearRatio;
    
    // Apply configuration
    motor.getConfigurator().apply(config);
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    // Step 1: Get the commanded voltage from motor and apply to simulation
    intakePivotSim.setInput(motor.getSimState().getMotorVoltage());
    
    // Step 2: Update the simulation by one timestep
    intakePivotSim.update(0.02);
    
    // Step 3: Update the motor sim state with the new simulated values
    motor.getSimState().setRawRotorPosition(Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
    motor.getSimState().setRotorVelocity(RadiansPerSecond.of(
        intakePivotSim.getVelocityRadPerSec() * gearRatio).in(RotationsPerSecond));

    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            intakePivotSim.getCurrentDrawAmps()));

    // Step 5: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.position = Radians.of(intakePivotSim.getAngleRads()).in(Rotations);
    inputs.velocity = RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.voltage = motor.getSimState().getMotorVoltage();
    inputs.statorCurrent = intakePivotSim.getCurrentDrawAmps();
    inputs.supplyCurrent = intakePivotSim.getCurrentDrawAmps();  // For arm, stator ≈ supply
  }

  /**
   * Set position using PID control with gravity compensation.
   * @param positionRotations Target position in rotations
   */
  public void setPosition(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  /**
   * Set velocity using closed-loop velocity control.
   * @param velocityRPS Target velocity in rotations per second
   */
  public void setVelocity(double velocityRPS) {
    motor.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setDutyCycle(double value) {
    motor.set(value);
  }

}
