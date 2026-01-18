// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationConstants;

public class IntakePivotIOSim implements IntakePivotIO {
  /** Creates a new IntakePivtoIOSim. */

  private double gearRatio = 1.0;

  private DCMotor gearbox = DCMotor.getKrakenX60(1);

  private final TalonFX motor = new TalonFX(SimulationConstants.INTAKE_PIVOT_MOTOR);

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
    intakePivotSim.setInput(motor.getSimState().getMotorVoltage());
    intakePivotSim.update(0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            intakePivotSim.getCurrentDrawAmps()));

    motor.getSimState().setRawRotorPosition(Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
    motor.getSimState().setRotorVelocity(RadiansPerSecond.of(
        intakePivotSim.getVelocityRadPerSec() * gearRatio).in(RotationsPerSecond));

    inputs.position = motor.getPosition().getValueAsDouble();
    inputs.velocity = motor.getVelocity().getValueAsDouble();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
  }

  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void setDutyCycle(double value) {
    motor.set(value);
  }

}
