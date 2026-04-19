// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FlywheelIOSim implements FlywheelIO {

  // Motor constants (defaults)
  private double gearRatio = 1.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(2);
  private final double flywheelMOI = 0.01; // kg*m^2

  private static final double KP = 5.0;
  private static final double KI = 0.0;
  private static final double KD = 0.01;

  // Disturbance injection for testing state machine
  private final LoggedNetworkBoolean injectDisturbance =
      new LoggedNetworkBoolean("/Tuning/Flywheel/InjectDisturbance", false);
  private final LoggedNetworkNumber disturbanceDurationSeconds =
      new LoggedNetworkNumber("/Tuning/Flywheel/DisturbanceDurationSeconds", 0.5);

  private double disturbanceStartTime = 0.0;
  private boolean disturbanceActive = false;

  // Single motor controller - gearbox is modeled as 2x Kraken X60 to retain combined power
  private final TalonFX motorController = new TalonFX(SimulationConstants.FLYWHEEL_MOTOR);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, flywheelMOI, gearRatio);
  private final FlywheelSim flywheelSim = new FlywheelSim(plant, gearbox, gearRatio);

  public FlywheelIOSim() {
    // Configure TalonFX with PID gains
    configureMotor();
    motorController
        .getSimState()
        .setRotorVelocity(
            RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0)
                .in(RotationsPerSecond));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kP = KP;
    slot0.kI = KI;
    slot0.kD = KD;

    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = 180.0;
    currentLimits.StatorCurrentLimitEnable = true;

    // Set coast mode (flywheel should coast when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration to motor
    motorController.getConfigurator().apply(talonConfig);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Step 1: Get the commanded voltage from the single motor controller.
    // The gearbox is modeled as 2x Kraken X60, so this voltage already represents
    // the combined effect of both physical motors on the shared flywheel mechanism.
    double motorVoltage = motorController.getSimState().getMotorVoltage();

    // Check if disturbance should be injected
    if (injectDisturbance.get() && !disturbanceActive) {
      disturbanceStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      disturbanceActive = true;
    }

    // Check if disturbance duration has elapsed
    if (disturbanceActive
        && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - disturbanceStartTime
            > disturbanceDurationSeconds.get()) {
      disturbanceActive = false;
      injectDisturbance.set(false); // Reset the toggle
    }

    // Apply disturbance: zero out the motor voltage during disturbance period
    double appliedVoltage = disturbanceActive ? 0.0 : motorVoltage;

    // Step 2: Update the simulation by one timestep
    flywheelSim.setInputVoltage(appliedVoltage);
    flywheelSim.update(SimulationConstants.SIM_TICK_RATE_S);

    // Convert RPM from simulation to RPS for consistency with Phoenix 6
    // (Phoenix 6 returns velocities in rotations per second by default)
    double velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;

    // Step 3: Update the motor sim state with the new simulated velocity
    motorController.getSimState().setRotorVelocity(velocityRPS);

    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    // Step 5: Read all inputs from the simulated values (source of truth).
    // Mirror readings to both slots since we model both motors as one.
    double statorCurrent = motorController.getStatorCurrent().getValueAsDouble();
    double supplyCurrent = motorController.getSupplyCurrent().getValueAsDouble();
    inputs.velocities[0] = flywheelSim.getAngularVelocityRPM();
    inputs.velocities[1] =
        flywheelSim.getAngularVelocityRPM(); // Both motors share the same shaft velocity
    inputs.voltages[0] = motorVoltage;
    inputs.voltages[1] = motorVoltage; // Same command sent to both physical motors
    inputs.statorCurrents[0] = statorCurrent;
    inputs.statorCurrents[1] = statorCurrent;
    inputs.supplyCurrents[0] = supplyCurrent;
    inputs.supplyCurrents[1] = supplyCurrent;
  }

  @Override
  public void setDutyCycle(double duty) {
    motorController.set(duty);
  }

  @Override
  public void setSpinupVelocityControl(double velocityRPM) {
    // Sim uses traditional PID control, not bang-bang
    motorController.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setHoldVelocityControl(double velocityRPM) {
    // Sim uses traditional PID control, not bang-bang
    // For simulation purposes, both methods use the same velocity control
    motorController.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setCoastVelocityControl(double velocityRPM) {
    // Traditional PID control (same as other methods in sim)
    motorController.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }
}
