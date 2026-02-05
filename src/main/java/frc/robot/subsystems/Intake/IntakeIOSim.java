// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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

public class IntakeIOSim implements IntakeIO {
  // Motor constants
  private double gearRatio = 1.0;
  private DCMotor gearbox = DCMotor.getNeo550(1);
  private final double moi = 0.0008; // Moment of inertia in kg·m²

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/Intake/SetpointRPM", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/Intake/Enabled", false);

  // Motor and control (using SparkFlex like the real hardware)
  private final SparkFlex motorControllerSim =
      new SparkFlex(SimulationConstants.INTAKE_MOTOR, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();

  // Sensor simulation
  private final DigitalInput sensor = new DigitalInput(SimulationConstants.INTAKE_SENSOR_PORT);
  private final DIOSim sensorSim = new DIOSim(sensor);

  // Flywheel simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, moi, gearRatio);
  private final FlywheelSim intakeSim = new FlywheelSim(plant, gearbox, gearRatio);

  public IntakeIOSim() {
    // Configure SparkMax with PID and current limits
    configureMotor();

    // Initialize everything to 0
    motorControllerSim.set(0.0);
  }

  private void configureMotor() {
    // Configure motor to match real hardware
    motorConfig.idleMode(IdleMode.kCoast); // Intake typically coasts
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(30);

    // Apply configuration
    motorControllerSim.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    // --- AdvantageScope tuning (sim-only) ---
    if (tuningEnabled.get()) {
      // Command the tunable setpoint
      double targetRPS = tunableSetpoint.get() / 60.0; // Convert RPM to RPS
      double targetDuty = targetRPS / 6000.0; // Assuming max ~6000 RPM for Neo550
      motorControllerSim.set(Math.max(-1, Math.min(1, targetDuty)));
    }

    // Step 1: Get the commanded voltage from motor controller
    double motorOutput = motorControllerSim.get();
    double appliedVoltage = motorOutput * 12.0; // Use 12V as bus voltage

    // Step 2: Update simulation
    intakeSim.setInputVoltage(appliedVoltage);
    intakeSim.update(0.02);

    // Step 3: Get simulated values directly
    double velocityRPM = intakeSim.getAngularVelocityRPM();
    double velocityRPS = velocityRPM / 60.0;

    // Step 4: Simple sensor simulation - triggers based on velocity
    sensorSim.setValue(Math.abs(velocityRPM) > 100); // Sensor triggers when spinning fast enough

    // Step 5: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));

    // Step 6: Set inputs
    inputs.velocity = velocityRPS;
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = intakeSim.getCurrentDrawAmps();
    inputs.supplyCurrent = intakeSim.getCurrentDrawAmps();
    inputs.sensor = sensorSim.getValue();
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }

  public void setVelocity(double velocity) {}

  @Override
  public void stop() {
    motorControllerSim.set(0.0);
  }
}
