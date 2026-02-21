// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private DoubleSupplier shootVelocitySupplier = () -> 0.0;
  private static final double TOLERANCE = 100.0;

  public enum FlywheelStates {
    OFF,
    AT_SETPOINT,
    MOVING
  }

  /**
   * Sets the supplier for the shoot velocity from the shot calculator.
   *
   * @param shootVelocitySupplier a DoubleSupplier providing the desired flywheel velocity in RPM
   */
  public void setShootVelocitySupplier(DoubleSupplier shootVelocitySupplier) {
    this.shootVelocitySupplier = shootVelocitySupplier;
  }

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public FlywheelStates getState() {
    return currentState;
  }

  private FlywheelStates wantedState = FlywheelStates.OFF;
  private FlywheelStates currentState = FlywheelStates.OFF;
  private FlywheelStates previousState = FlywheelStates.OFF;

  public boolean atSetpoint(double targetRPM, double tolerance) {
    // TODO: make tolerance a constant in hardware layer
    return Math.abs(inputs.velocities[0] - targetRPM) < tolerance;
  }

  public boolean atSetpoint(DoubleSupplier targetRPM, double tolerance) {
    return atSetpoint(targetRPM.getAsDouble(), tolerance);
  }

  private void updateState() {
    previousState = currentState;

    // State machine transitions
    switch (wantedState) {
      case MOVING:
        if (atSetpoint(shootVelocitySupplier.getAsDouble(), TOLERANCE)) {
          currentState = FlywheelStates.AT_SETPOINT;
        } else {
          currentState = FlywheelStates.MOVING;
        }
        break;
      case OFF:
      default:
        currentState = FlywheelStates.OFF;
        break;
    }
  }

  public double getLeaderVelocityRPS() {
    if (inputs.velocities.length > 0) {
      return inputs.velocities[0]; // Convert from RPM to RPS
    }
    return 0.0;
  }

  private void applyState() {
    switch (currentState) {
      case MOVING:
      case AT_SETPOINT:
        setVelocity(shootVelocitySupplier.getAsDouble());
        break;
      case OFF:
      default:
        io.setDutyCycle(0.0);
        break;
    }
  }

  public void setWantedState(FlywheelStates state) {
    wantedState = state;
    // State update will happen in periodic()
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // Update state machine on every cycle to respond to velocity/current state changes
    updateState();
    applyState();

    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState.toString());
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void setVelocity(double rpm) {
    io.setVelocity(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double rpm) {
    return this.runEnd(() -> setVelocity(rpm), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(
        () -> setVelocity(supplierVelocity.getAsDouble()), () -> io.setDutyCycle(0.0));
  }
}
