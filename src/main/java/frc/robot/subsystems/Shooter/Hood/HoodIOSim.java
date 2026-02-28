// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.SimulationConstants;


public class HoodIOSim implements HoodIO {
  // Match PracticeBot (PB) config
  private static final double GEAR_RATIO = 3.0 / 1.0 * 170.0 / 10.0;
  private static final double KP = 300.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KA = 0.0;
  private static final double KG = 0.15;
  private static final double KS = 0.2;
  private static final double KV = 0.0;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 25.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
  private static final double ARM_LENGTH = 0.3; // meters (12 inches)
  private static final double ARM_MASS = 1.5; // kg
  private static final double FORWARD_SOFT_LIMIT_DEGREES = 47.0;
  private static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0;

  // Motor and control
  private final TalonFX motorControllerSim = new TalonFX(SimulationConstants.HOOD_MOTOR);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  // Simulation
  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final SingleJointedArmSim hoodSim =
    new SingleJointedArmSim(
      gearbox,
      GEAR_RATIO,
      SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS),
      ARM_LENGTH,
      Units.degreesToRadians(REVERSE_SOFT_LIMIT_DEGREES),
      Units.degreesToRadians(FORWARD_SOFT_LIMIT_DEGREES),
      true, // Simulate gravity
      0);

  public HoodIOSim() {
    // Configure TalonFX with PID and gravity compensation
    configureMotor();

    // Initialize motor sim state to match arm sim initial position
    motorControllerSim
        .getSimState()
        .setRawRotorPosition(Radians.of(hoodSim.getAngleRads() * GEAR_RATIO).in(Rotations));
  }

  private void configureMotor() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Configure PID gains for slot 0 (match PB)
    Slot0Configs slot0 = talonConfig.Slot0;
    slot0.kP = KP;
    slot0.kI = KI;
    slot0.kD = KD;
    slot0.kA = KA;
    slot0.kG = KG;
    slot0.kS = KS;
    slot0.kV = KV;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Configure current limits for safety
    CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    currentLimits.SupplyCurrentLimitEnable = true;

    // Set brake mode (hold position when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Apply configuration
    motorControllerSim.getConfigurator().apply(talonConfig);
  }

  public void updateInputs(HoodIOInputs inputs) {
    // Step 1: Get the commanded voltage from motor and apply to simulation
    hoodSim.setInput(motorControllerSim.getSimState().getMotorVoltage());

    // Step 2: Update the simulation by one timestep
    hoodSim.update(SimulationConstants.SIM_TICK_RATE_S);

    // Step 3: Update the motor sim state with the new simulated values
    motorControllerSim
        .getSimState()
        .setRawRotorPosition(Radians.of(hoodSim.getAngleRads() * GEAR_RATIO).in(Rotations));

    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.getCurrentDrawAmps()));

    // Step 5: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.position = Units.radiansToDegrees(hoodSim.getAngleRads());
    inputs.velocity = Units.radiansToDegrees(hoodSim.getVelocityRadPerSec());
    inputs.voltage = motorControllerSim.getSimState().getMotorVoltage();
    inputs.statorCurrent = motorControllerSim.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motorControllerSim.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Set position in degrees (matches PB IO)
   */
  @Override
  public void setPosition(double positionDegrees) {
    // Convert degrees to rotations for the sim motor
    double rotations = positionDegrees / 360.0;
    motorControllerSim.setControl(positionRequest.withPosition(rotations));
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }

  @Override
  public void setZero() {
    motorControllerSim.getSimState().setRawRotorPosition(0.0);
  }
}
