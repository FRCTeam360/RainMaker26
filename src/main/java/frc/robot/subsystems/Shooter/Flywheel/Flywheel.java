// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // Constants
  private static final double TOLERANCE_RPM = 100.0;
  private static final double CONTROL_MODE_DEBOUNCE_SECONDS = 0.04;
  private static final double AT_GOAL_DEBOUNCE_SECONDS = 0.2;

  private boolean atGoal = false;
  private long launchCount = 0;

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
    SPINNING_UP,
    AT_SETPOINT
  }

  // controlModeDebouncer: prevents rapid spinup/hold switching when velocity briefly dips below
  // tolerance due to noise or disturbances that aren't a ball pass-through
  private final Debouncer controlModeDebouncer =
      new Debouncer(CONTROL_MODE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  // shooterSetpointDebouncer: requires velocity to be in tolerance for a sustained period before
  // reverting back to preparing to shoot shoot state, filtering out momentary spikes
  private final Debouncer shooterSetpointDebouncer =
      new Debouncer(AT_GOAL_DEBOUNCE_SECONDS, DebounceType.kFalling);

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
          if (previousState == FlywheelInternalStates.AT_SETPOINT) {
            launchCount++;
          }
          currentState = FlywheelInternalStates.SPINNING_UP;
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
      case SPINNING_UP:
        setSpinupVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case AT_SETPOINT:
        setHoldVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case OFF:
      default:
        atGoal = false;
        setDutyCycle(0.0);
        break;
    }
  }

  private boolean atSetpoint(double targetRPM) {
    boolean controlModeAtSetpoint;
    boolean inTolerance;
    if (inputs.velocities.length > 0) {
      inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
      controlModeAtSetpoint = controlModeDebouncer.calculate(inTolerance);
      atGoal = shooterSetpointDebouncer.calculate(inTolerance);
      return controlModeAtSetpoint;
    }
    return false;
  }

  public boolean isReadyToShoot() {
    return atGoal;
  }

  // IO delegation methods

  private void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void setSpinupVelocityControl(double rpm) {
    io.setSpinupVelocityControl(rpm);
  }

  public void setHoldVelocityControl(double rpm) {
    io.setHoldVelocityControl(rpm);
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
    return this.runEnd(() -> setHoldVelocityControl(rpm), () -> setDutyCycle(0.0));
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(
        () -> setHoldVelocityControl(supplierVelocity.getAsDouble()), () -> setDutyCycle(0.0));
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

    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState);
    Logger.recordOutput("Subsystems/Flywheel/ControlState", controlState);
    Logger.recordOutput("Subsystems/Flywheel/LaunchCount", launchCount);
  }
}
