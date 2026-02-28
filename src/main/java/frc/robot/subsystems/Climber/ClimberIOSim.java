// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;

public class ClimberIOSim implements ClimberIO {
  // Physical constants
  private static final double GEAR_RATIO = 20.0; // gear reduction
  private static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75); // 3/4 in drum
  private static final double CARRIAGE_MASS_KG = 4.5; // carriage + arm mass
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = Units.inchesToMeters(24.0); // 24 in travel
  private static final double MIN_HEIGHT_THRESHOLD_METERS = 0.01;

  // Simulated motor controller (single climber sim, reports identical values to both sides)
  private final SparkMax motorControllerSim =
      new SparkMax(SimulationConstants.CLIMBER_MOTOR, MotorType.kBrushless);
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // Elevator simulation
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final ElevatorSim climberSim =
      new ElevatorSim(
          gearbox,
          GEAR_RATIO,
          CARRIAGE_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_HEIGHT_METERS,
          MAX_HEIGHT_METERS,
          true,
          MIN_HEIGHT_METERS);

  /** Creates a new ClimberIOSim. */
  public ClimberIOSim() {
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);
    motorControllerSim.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorControllerSim.set(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Step 1: Get commanded voltage
    double motorOutput = motorControllerSim.get();
    double appliedVoltage = motorOutput * 12.0;

    // Step 2: Update elevator simulation
    climberSim.setInputVoltage(appliedVoltage);
    climberSim.update(SimulationConstants.SIM_TICK_RATE_S);

    // Step 3: Read simulated outputs
    double positionMeters = climberSim.getPositionMeters();
    double velocityMPS = climberSim.getVelocityMetersPerSecond();
    double currentAmps = climberSim.getCurrentDrawAmps();

    // Step 4: Update battery sim
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentAmps));

    // Step 5: Report identical values to both left and right inputs
    inputs.climberLeftPosition = positionMeters;
    inputs.climberRightPosition = positionMeters;
    inputs.climberLeftVelocity = velocityMPS;
    inputs.climberRightVelocity = velocityMPS;
    inputs.climberLeftDutyCycle = motorOutput;
    inputs.climberRightDutyCycle = motorOutput;
    inputs.climberLeftCurrent = currentAmps;
    inputs.climberRightCurrent = currentAmps;
    inputs.climberLeftTemp = 0.0;
    inputs.climberRightTemp = 0.0;
  }

  @Override
  public void setLeftDutyCycle(double dutyCycle) {
    motorControllerSim.set(dutyCycle);
  }

  @Override
  public void setRightDutyCycle(double dutyCycle) {
    motorControllerSim.set(dutyCycle);
  }

  @Override
  public void setLeftPosition(double position) {}

  @Override
  public void setRightPosition(double position) {}

  @Override
  public void zeroBoth() {
    climberSim.setState(0.0, 0.0);
    motorControllerSim.set(0.0);
  }
}
