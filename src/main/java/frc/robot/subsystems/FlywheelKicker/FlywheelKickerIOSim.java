// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FlywheelKicker;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FlywheelKickerIOSim implements FlywheelKickerIO {

  // Motor constants (defaults)
  private double gearRatio = 1.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final double flywheelMOI = 0.005; // kg*m^2 (smaller than main flywheel)

  // AdvantageScope tuning (sim-only, under /Tuning table)
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
  private final TalonFX motorControllerSim = new TalonFX(SimulationConstants.FLYWHEEL_KICKER_MOTOR);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Sensor simulation
  private final DigitalInput sensor =
      new DigitalInput(SimulationConstants.FLYWHEEL_KICKER_SENSOR_ID);
  private final DIOSim sensorSim = new DIOSim(sensor);

  // Simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, flywheelMOI, gearRatio);
  private final FlywheelSim flywheelKickerSim = new FlywheelSim(plant, gearbox, gearRatio);

  // Angular position tracking
  private double angularPositionRotations = 0.0;

  public FlywheelKickerIOSim() {
    // Configure TalonFX with PID gains
    configureMotor();

    // Initialize motor sim state to match flywheel sim initial state
    motorControllerSim
        .getSimState()
        .setRotorVelocity(
            RotationsPerSecond.of(flywheelKickerSim.getAngularVelocityRPM() / 60.0)
                .in(RotationsPerSecond));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Configure PID gains for slot 0 (use tunable defaults)
    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kP = tunableKp.get();
    slot0.kI = tunableKi.get();
    slot0.kD = tunableKd.get();

    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = 120.0;
    currentLimits.StatorCurrentLimitEnable = true;

    // Set coast mode (flywheel kicker should coast when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motorControllerSim.getConfigurator().apply(talonConfig);
  }

  @Override
  public void updateInputs(FlywheelKickerIOInputs inputs) {
    // --- AdvantageScope tuning (sim-only) ---
    if (tuningEnabled.get()) {
      // Apply tunable PID gains (simple: apply every loop when enabled)
      Slot0Configs slot0 = new Slot0Configs();
      motorControllerSim.getConfigurator().refresh(slot0);
      slot0.kP = tunableKp.get();
      slot0.kI = tunableKi.get();
      slot0.kD = tunableKd.get();
      motorControllerSim.getConfigurator().apply(slot0);

      // Command the tunable setpoint
      double targetRPS = tunableSetpoint.get() / 60.0;
      motorControllerSim.setControl(velocityRequest.withVelocity(targetRPS));
    }

    // Step 1: Get the commanded voltage from motor and apply to simulation
    double motorVoltage = motorControllerSim.getSimState().getMotorVoltage();
    flywheelKickerSim.setInputVoltage(motorVoltage);

    // Step 2: Update the simulation by one timestep
    flywheelKickerSim.update(0.02);

    // Step 3: Update angular position by integrating velocity
    double velocityRPS = flywheelKickerSim.getAngularVelocityRPM() / 60.0;
    angularPositionRotations += velocityRPS * 0.02; // Integrate velocity over time

    // Step 4: Update the motor sim state with the new simulated values
    motorControllerSim.getSimState().setRawRotorPosition(angularPositionRotations * gearRatio);
    motorControllerSim
        .getSimState()
        .setRotorVelocity(RotationsPerSecond.of(velocityRPS).in(RotationsPerSecond));

    // Step 5: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelKickerSim.getCurrentDrawAmps()));

    // Step 6: Simple sensor simulation - triggers when flywheel is at speed
    sensorSim.setValue(
        Math.abs(flywheelKickerSim.getAngularVelocityRPM()) > 2000); // Sensor triggers at high RPM

    // Step 7: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.velocity = velocityRPS;
    inputs.voltage = motorVoltage;
    inputs.statorCurrent = motorControllerSim.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motorControllerSim.getSupplyCurrent().getValueAsDouble();
    inputs.sensor = sensorSim.getValue();
  }

  @Override
  public void setDutyCycle(double duty) {
    motorControllerSim.set(duty);
  }
}
