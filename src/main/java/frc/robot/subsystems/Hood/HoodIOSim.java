// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodIOSim implements HoodIO {

  // Motor constants (defaults)
  private double gearRatio = 15.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  private final double kG = 0.5; // Gravity compensation
  private final double armLength = 0.3; // meters (12 inches)
  private final double armMass = 1.5; // kg

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber tunableKp = new LoggedNetworkNumber("/Tuning/Hood/kP", 10.0);
  private final LoggedNetworkNumber tunableKi = new LoggedNetworkNumber("/Tuning/Hood/kI", 0.0);
  private final LoggedNetworkNumber tunableKd = new LoggedNetworkNumber("/Tuning/Hood/kD", 0.8);
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/Hood/SetpointRotations", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/Hood/Enabled", false);

  // Motor and control
  private final TalonFX motorControllerSim = new TalonFX(SimulationConstants.HOOD_MOTOR);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  // Simulation
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          gearbox,
          gearRatio,
          SingleJointedArmSim.estimateMOI(armLength, armMass),
          armLength,
          Units.degreesToRadians(0),
          Units.degreesToRadians(30),
          true, // Simulate gravity
          0);

  public HoodIOSim() {
    // Configure TalonFX with PID and gravity compensation
    configureMotor();

    // Initialize motor sim state to match arm sim initial position
    motorControllerSim
        .getSimState()
        .setRawRotorPosition(Radians.of(hoodSim.getAngleRads() * gearRatio).in(Rotations));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Configure PID gains for slot 0 (use tunable defaults)
    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kP = tunableKp.get();
    slot0.kI = tunableKi.get();
    slot0.kD = tunableKd.get();
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine; // Gravity compensation

    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = 120.0;
    currentLimits.StatorCurrentLimitEnable = true;

    // Set brake mode (hold position when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motorControllerSim.getConfigurator().apply(talonConfig);
  }

  public void updateInputs(HoodIOInputs inputs) {
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
      motorControllerSim.setControl(positionRequest.withPosition(tunableSetpoint.get()));
    }

    // Step 1: Get the commanded voltage from motor and apply to simulation
    hoodSim.setInput(motorControllerSim.getSimState().getMotorVoltage());

    // Step 2: Update the simulation by one timestep
    hoodSim.update(0.02);

    // Step 3: Update the motor sim state with the new simulated values
    motorControllerSim
        .getSimState()
        .setRawRotorPosition(Radians.of(hoodSim.getAngleRads() * gearRatio).in(Rotations));

    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.getCurrentDrawAmps()));

    // Step 5: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.position = Radians.of(hoodSim.getAngleRads()).in(Rotations);
    inputs.voltage = motorControllerSim.getSimState().getMotorVoltage();
    inputs.statorCurrent = motorControllerSim.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motorControllerSim.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Set position using PID control with gravity compensation.
   *
   * @param positionRotations Target position in rotations
   */
  @Override
  public void setPosition(double positionRotations) {
    motorControllerSim.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }

  @Override
  public void setEncoder(double position) {
    motorControllerSim.getSimState().setRawRotorPosition(position);
  }
}
