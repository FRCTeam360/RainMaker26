// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // Constants
  private static final double TOLERANCE_RPM = 100.0;

  // IO fields
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Other fields
  private DoubleSupplier shootVelocitySupplier = () -> 0.0;

  // Enums
  public enum FlywheelWantedStates {
    IDLE,
    SHOOTING
  }

  public enum FlywheelInternalStates {
    OFF,
    MOVING,
    AT_SETPOINT
  }

  // State variables
  private FlywheelWantedStates wantedState = FlywheelWantedStates.IDLE;
  private FlywheelInternalStates currentState = FlywheelInternalStates.OFF;
  private FlywheelInternalStates previousState = FlywheelInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  // State machine methods

  public FlywheelInternalStates getState() {
    return currentState;
  }

  public void setWantedState(FlywheelWantedStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  /**
   * Sets the supplier for the shoot velocity from the shot calculator.
   *
   * @param shootVelocitySupplier a DoubleSupplier providing the desired flywheel velocity in RPM
   */
  public void setShootVelocitySupplier(DoubleSupplier shootVelocitySupplier) {
    this.shootVelocitySupplier = shootVelocitySupplier;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING:
        if (atSetpoint(shootVelocitySupplier.getAsDouble())) {
          currentState = FlywheelInternalStates.AT_SETPOINT;
        } else {
          currentState = FlywheelInternalStates.MOVING;
        }
        break;
      case IDLE:
      default:
        currentState = FlywheelInternalStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case MOVING:
      case AT_SETPOINT:
        setVelocity(shootVelocitySupplier.getAsDouble());
        break;
      case OFF:
      default:
        setDutyCycle(0.0);
        break;
    }
  }

  private boolean atSetpoint(double targetRPM) {
    // TODO: make tolerance a constant in hardware layer
    if (inputs.velocities.length > 0) {
      return Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
    }
    return false;
  }

  private boolean atSetpoint(DoubleSupplier targetRPM) {
    return atSetpoint(targetRPM.getAsDouble());
  }

  // IO delegation methods

  private void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void setVelocity(double rpm) {
    io.setVelocity(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> setDutyCycle(valueSup.getAsDouble()), () -> setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double rpm) {
    return this.runEnd(() -> setVelocity(rpm), () -> setDutyCycle(0.0));
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(() -> setVelocity(supplierVelocity.getAsDouble()), () -> setDutyCycle(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      // Update state machine on every cycle to respond to velocity/current state changes
      updateState();
      applyState();
    }

    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState.toString());
    Logger.recordOutput("Subsystems/Flywheel/ControlState", controlState.toString());
  }
}
