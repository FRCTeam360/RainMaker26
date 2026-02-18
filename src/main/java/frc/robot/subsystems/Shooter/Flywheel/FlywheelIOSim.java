// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
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

  private final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kP", 2.0);
  private final LoggedNetworkNumber tunableKi =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kI", 0.0);
  private final LoggedNetworkNumber tunableKd =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/kD", 0.1);
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/FlywheelKicker/SetpointRPM", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/FlywheelKicker/Enabled", false);

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

    motorControllerSim2.setControl(new StrictFollower(SimulationConstants.FLYWHEEL_MOTOR));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kP = tunableKp.get();
    slot0.kI = tunableKi.get();
    slot0.kD = tunableKd.get();

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
      Slot0Configs slot0 = new Slot0Configs();
      motorControllerSim1.getConfigurator().refresh(slot0);
      slot0.kP = tunableKp.get();
      slot0.kI = tunableKi.get();
      slot0.kD = tunableKd.get();
      motorControllerSim1.getConfigurator().apply(slot0);

      // Command the tunable setpoint to both motors
      this.setVelocity(tunableSetpoint.get());
    }

    // Step 1: Get the commanded voltage from motors and apply to simulation
    // Use average voltage from both motors for the flywheel sim
    // TODO: Verify StrictFollower works in simulation - check if motorVoltage2 tracks
    // motorVoltage1.
    //       If motorVoltage2 is always 0, the follower isn't working and we need to command both
    //       motors directly or use motorVoltage1 * 2 for the physics sim.
    double motorVoltage1 = motorControllerSim1.getSimState().getMotorVoltage();
    double motorVoltage2 = motorControllerSim2.getSimState().getMotorVoltage();
    double averageVoltage = (motorVoltage1 + motorVoltage2) / 2.0;
    flywheelSim.setInputVoltage(averageVoltage);

    // Step 2: Update the simulation by one timestep
    flywheelSim.update(SimulationConstants.SIM_TICK_RATE_S);

    double velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;

    // Step 4: Update the motor sim states with the new simulated values
    motorControllerSim1.getSimState().setRotorVelocity(velocityRPS);
    motorControllerSim2.getSimState().setRotorVelocity(velocityRPS);

    // Step 5: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    // Step 6: Read all inputs from the SIMULATED VALUES (source of truth)
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
  }

  @Override
  public void setVelocity(double rpm) {
    rpm = rpm / 60;
    motorControllerSim1.setControl(velocityRequest.withVelocity(rpm));
  }
}
