// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakePivotIOPB implements IntakePivotIO {

  // Motor constants (defaults)
  private double gearRatio = 10.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  private final double kG = 0.75; // Gravity compensation
  private final double armLength = 0.762; // meters (30 inches)
  private final double armMass = 2.0; // kg

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("/Tuning/IntakePivot/kP", 7.0);
  private final LoggedNetworkNumber tunableKi =
      new LoggedNetworkNumber("/Tuning/IntakePivot/kI", 0.0);
  private final LoggedNetworkNumber tunableKd =
      new LoggedNetworkNumber("/Tuning/IntakePivot/kD", 0.5);
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/IntakePivot/SetpointRotations", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/IntakePivot/Enabled", false);

  // Motor and control
  private final TalonFX motorControllerSim = new TalonFX(SimulationConstants.INTAKE_PIVOT_MOTOR);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public IntakePivotIOPB() {
    // Configure TalonFX with PID and gravity compensation
    configureMotor();
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
    currentLimits.StatorCurrentLimit = 180.0;
    currentLimits.StatorCurrentLimitEnable = true;

    // Set brake mode (hold position when no command)
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply sensor-to-mechanism ratio for cleaner encoder readings
    talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motorControllerSim.getConfigurator().apply(talonConfig);
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
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
  }

  /**
   * Set position using PID control with gravity compensation.
   *
   * @param positionRotations Target position in rotations
   */
  public void setPosition(double positionRotations) {
    motorControllerSim.setControl(positionRequest.withPosition(positionRotations));
  }

  /**
   * Set velocity using closed-loop velocity control.
   *
   * @param velocityRPS Target velocity in rotations per second
   */
  public void setVelocity(double velocityRPS) {
    motorControllerSim.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }
}
