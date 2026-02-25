// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperRollerIOSim implements HopperRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final double MOI = 0.0008; // kg·m²

  private final DCMotor gearbox = DCMotor.getNeoVortex(1);
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(gearbox, MOI, GEAR_RATIO), gearbox, GEAR_RATIO);

  private double appliedDutyCycle = 0.0;

  @Override
  public void updateInputs(HopperRollerIOInputs inputs) {
    sim.setInputVoltage(appliedDutyCycle * 12.0);
    sim.update(0.02);

    inputs.velocity = sim.getAngularVelocityRPM() / 60.0; // RPS
    inputs.voltage = appliedDutyCycle * 12.0;
    inputs.statorCurrent = sim.getCurrentDrawAmps();
    inputs.supplyCurrent = sim.getCurrentDrawAmps();
    inputs.position = 0.0; // not tracked in sim
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    appliedDutyCycle = dutyCycle;
  }
}
