// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

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
  // private final double kS = 0.0;
  // private final double kV = 0.0;
  // private final double kA = 0.0;
  private final double flywheelMOI = 0.01; // kg*m^2

  // AdvantageScope tuning (sim-only, under /Tuning table)
  // private final LoggedNetworkNumber tunableKp = new LoggedNetworkNumber("/Tuning/Flywheel/kP",
  // 1.0);
  // private final LoggedNetworkNumber tunableKi = new LoggedNetworkNumber("/Tuning/Flywheel/kI",
  // 0.0);
  // private final LoggedNetworkNumber tunableKd = new LoggedNetworkNumber("/Tuning/Flywheel/kD",
  // 0.0);
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/Flywheel/SetpointRPM", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/Flywheel/Enabled", false);

  // Motor and control
  private final TalonFX motorControllerSim1 = new TalonFX(SimulationConstants.FLYWHEEL_MOTOR);
  private final TalonFX motorControllerSim2 = new TalonFX(SimulationConstants.FLYWHEEL_MOTOR + 1);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, flywheelMOI, gearRatio);
  private final FlywheelSim flywheelSim = new FlywheelSim(plant, gearbox, gearRatio);

  public FlywheelIOSim() {
    // Configure TalonFX with PID gains
    configureMotor();

    // Initialize motor sim state to match flywheel sim initial state
    // motorControllerSim1.getSimState().setRawRotorPosition(
    //     Radians.of(flywheelSim.() * gearRatio).in(Rotations));
    // motorControllerSim2.getSimState().setRawRotorPosition(angularPositionRotations);
    motorControllerSim1
        .getSimState()
        .setRotorVelocity(
            RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0)
                .in(RotationsPerSecond));
    motorControllerSim2
        .getSimState()
        .setRotorVelocity(
            RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0)
                .in(RotationsPerSecond));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Configure PID gains for slot 0 (use tunable defaults)
    // Slot0Configs slot0 = talonConfig.Slot0;
    // slot0.kP = tunableKp.get();
    // slot0.kI = tunableKi.get();
    // slot0.kD = tunableKd.get();
    // slot0.kS = kS;
    // slot0.kV = kV;
    // slot0.kA = kA;

    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = 180.0;
    currentLimits.StatorCurrentLimitEnable = true;

    // Set coast mode (flywheel should coast when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration to both motors
    motorControllerSim1.getConfigurator().apply(talonConfig);
    motorControllerSim2.getConfigurator().apply(talonConfig);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    // --- AdvantageScope tuning (sim-only) ---
    if (tuningEnabled.get()) {
      // Apply tunable PID gains (simple: apply every loop when enabled)
      Slot0Configs slot0 = new Slot0Configs();
      motorControllerSim1.getConfigurator().refresh(slot0);
      // slot0.kP = tunableKp.get();
      // slot0.kI = tunableKi.get();
      // slot0.kD = tunableKd.get();
      motorControllerSim1.getConfigurator().apply(slot0);
      motorControllerSim2.getConfigurator().apply(slot0);

      // Command the tunable setpoint to both motors
      double targetRPS = tunableSetpoint.get() / 60.0;
      motorControllerSim1.setControl(velocityRequest.withVelocity(targetRPS));
      motorControllerSim2.setControl(velocityRequest.withVelocity(targetRPS));
    }

    // Step 1: Get the commanded voltage from motors and apply to simulation
    // Use average voltage from both motors for the flywheel sim
    double motorVoltage1 = motorControllerSim1.getSimState().getMotorVoltage();
    double motorVoltage2 = motorControllerSim2.getSimState().getMotorVoltage();
    double averageVoltage = (motorVoltage1 + motorVoltage2) / 2.0;
    flywheelSim.setInputVoltage(averageVoltage);

    // Step 2: Update the simulation by one timestep
    flywheelSim.update(0.02);

    // Step 3: Update angular position by integrating velocity
    double velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
    // angularPositionRotations += velocityRPS * 0.02; // Integrate velocity over time

    // Step 4: Update the motor sim states with the new simulated values
    //  motorControllerSim1.getSimState().setRawRotorPosition(angularPositionRotations);
    motorControllerSim1
        .getSimState()
        .setRotorVelocity(RotationsPerSecond.of(velocityRPS).in(RotationsPerSecond));
    // motorControllerSim2.getSimState().setRawRotorPosition(angularPositionRotations);
    motorControllerSim2
        .getSimState()
        .setRotorVelocity(RotationsPerSecond.of(velocityRPS).in(RotationsPerSecond));

    // Step 5: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    // Step 6: Read all inputs from the SIMULATED VALUES (source of truth)
    // inputs.positions[0] = angularPositionRotations;
    // inputs.positions[1] = angularPositionRotations; // Both motors have same position
    inputs.velocities[0] = velocityRPS;
    inputs.velocities[1] = velocityRPS; // Both motors have same velocity
    inputs.voltages[0] = motorVoltage1;
    inputs.voltages[1] = motorVoltage2;
    inputs.statorCurrents[0] = motorControllerSim1.getStatorCurrent().getValueAsDouble();
    inputs.statorCurrents[1] = motorControllerSim2.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrents[0] = motorControllerSim1.getSupplyCurrent().getValueAsDouble();
    inputs.supplyCurrents[1] = motorControllerSim2.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setDutyCycle(double duty) {
    motorControllerSim1.set(duty);
    motorControllerSim2.set(duty);
  }

  @Override
  public void setRPM(double rpm) {
    motorControllerSim1.setControl(velocityRequest.withVelocity(rpm));
    motorControllerSim2.setControl(velocityRequest.withVelocity(rpm));
  }
}
