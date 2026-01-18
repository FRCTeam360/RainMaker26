// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.hardware.TalonFX;

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
  /** Creates a new IntakePivtoIOSim. */

  private double gearRatio = 1.0;

  private DCMotor gearbox = DCMotor.getKrakenX60(1);

  private final TalonFX motor = new TalonFX(22);

  private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
      gearbox,
      gearRatio,
      SingleJointedArmSim.estimateMOI(
          Units.inchesToMeters(3), 2.0),
      Units.inchesToMeters(30),
      Units.degreesToRadians(-75),
      Units.degreesToRadians(255),
      true,
      0);

  public IntakePivotIOSim() {
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    // Step 1: Get the command voltage from the motor and apply to simulation
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

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void setDutyCycle(double value) {
    motor.set(value);
  }

}
