// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {

  // FIXME: set actual gear ratio, drum radius, carriage mass, and min/max height
  private static final double GEAR_RATIO = 20.0;
  private static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
  private static final double CARRIAGE_MASS_KG = 4.0;
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = Units.inchesToMeters(24.0);
  private static final double STARTING_HEIGHT_METERS = 0.0;

  // FIXME: tune PID gains for sim
  private static final double SIM_KP = 100.0;
  private static final double SIM_KI = 0.0;
  private static final double SIM_KD = 1.0;

  private final DCMotor gearbox = DCMotor.getNEO(1);

  // Single sim — values are mirrored to both left and right inputs
  private final ElevatorSim climberSim =
      new ElevatorSim(
          gearbox,
          GEAR_RATIO,
          CARRIAGE_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_HEIGHT_METERS,
          MAX_HEIGHT_METERS,
          true, // simulate gravity
          STARTING_HEIGHT_METERS);

  private final PIDController pid = new PIDController(SIM_KP, SIM_KI, SIM_KD);

  private double leftDutyCycle = 0.0;

  // null = duty cycle mode, non-null = position mode
  private Double positionSetpointMeters = null;

  /** Creates a new ClimberIOSim. */
  public ClimberIOSim() {
    pid.setTolerance(0.01); // 1 cm
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    double appliedVoltage;

    if (positionSetpointMeters != null) {
      // Position mode: use PID to compute voltage
      double pidOutput = pid.calculate(climberSim.getPositionMeters(), positionSetpointMeters);
      appliedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    } else {
      // Duty cycle mode
      appliedVoltage = leftDutyCycle * 12.0;
    }

    climberSim.setInput(appliedVoltage);
    climberSim.update(0.02);

    double position = climberSim.getPositionMeters();
    double velocity = climberSim.getVelocityMetersPerSecond();
    double current = climberSim.getCurrentDrawAmps();

    // Mirror single sim to both sides
    inputs.climberLeftPosition = position;
    inputs.climberRightPosition = position;
    inputs.climberLeftVelocity = velocity;
    inputs.climberRightVelocity = velocity;
    inputs.climberLeftDutyCycle = leftDutyCycle;
    inputs.climberRightDutyCycle = leftDutyCycle;
    inputs.climberLeftCurrent = current;
    inputs.climberRightCurrent = current;
    inputs.climberLeftTemp = 0.0;
    inputs.climberRightTemp = 0.0;
  }

  @Override
  public void setLeftDutyCycle(double dutyCycle) {
    positionSetpointMeters = null;
    leftDutyCycle = dutyCycle;
  }

  @Override
  public void setRightDutyCycle(double dutyCycle) {
    positionSetpointMeters = null;
    leftDutyCycle = dutyCycle;
  }

  @Override
  public void setLeftPosition(double position) {
    positionSetpointMeters = position;
  }

  @Override
  public void setRightPosition(double position) {
    positionSetpointMeters = position;
  }

  @Override
  public boolean leftAboveMinHeight() {
    return climberSim.getPositionMeters() > MIN_HEIGHT_METERS;
  }

  @Override
  public boolean rightAboveMinHeight() {
    return climberSim.getPositionMeters() > MIN_HEIGHT_METERS;
  }

  @Override
  public void zeroBoth() {
    positionSetpointMeters = null;
    leftDutyCycle = 0.0;
  }

  @Override
  public void updatePIDF(double P, double I, double D, double F) {
    pid.setPID(P, I, D);
  }
}


