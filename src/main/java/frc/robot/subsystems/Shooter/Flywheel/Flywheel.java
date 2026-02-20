// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public enum FlywheelStates {
    OFF,
    SPINNING_UP, // Duty cycle bang-bang - fast acceleration
    AT_SETPOINT, // Torque current bang-bang - maintaining speed
    COAST // Legacy state for compatibility
  }

  // These enums are for logging and debugging - actual control mode is determined by state machine
  public enum FlywheelControlType {
    DUTY_CYCLE_BANG_BANG, // Startup/Recovery - fast acceleration
    TORQUE_CURRENT_BANG_BANG, // Idle/Ball - consistent torque
    VOLTAGE_VELOCITY, // Velocity PID control
    STOP
  }

  // Tunable parameters for 4-phase bang-bang control
  private static final LoggedNetworkNumber toleranceRPS =
      new LoggedNetworkNumber("Flywheel/ToleranceRPS", 3); // 0.33 RPS ≈ 20 RPM
  private static final LoggedNetworkNumber controlModeDebounceSeconds =
      new LoggedNetworkNumber("Flywheel/ControlModeDebounceSeconds", 0.025);
  private static final LoggedNetworkNumber atGoalDebounceSeconds =
      new LoggedNetworkNumber("Flywheel/AtGoalDebounceSeconds", 0.2);

  private final Debouncer controlModeDebouncer =
      new Debouncer(controlModeDebounceSeconds.get(), DebounceType.kFalling);
  private final Debouncer atGoalDebouncer =
      new Debouncer(atGoalDebounceSeconds.get(), DebounceType.kFalling);

  private FlywheelControlType currentControlMode = FlywheelControlType.STOP;
  private boolean atGoal = false;
  private double targetVelocityRPS = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  /**
   * Set flywheel velocity using 4-phase bang-bang control. State machine handles control mode
   * transitions automatically. Can be called repeatedly with different targets - state machine will
   * adapt.
   *
   * @param velocityRPM target velocity in rotations per minute (RPM). Internally converted to RPS
   *     for IO layer which expects rotations per second.
   */
  public void setVelocityRPM(double velocityRPM) {
    // Convert RPM to RPS for internal IO layer communication
    // (Phoenix 6 velocities are in RPS, tolerance is in RPS)
    targetVelocityRPS = velocityRPM / 60.0;

    // Trigger state transition if starting or stopping
    if (velocityRPM > 0.0 && wantedState == FlywheelStates.OFF) {
      wantedState = FlywheelStates.SPINNING_UP;
    } else if (velocityRPM == 0.0) {
      wantedState = FlywheelStates.OFF;
    }
    // State machine in periodic() will handle remaining transitions (SPINNING_UP <-> AT_SETPOINT)
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public FlywheelControlType getControlMode() {
    return currentControlMode;
  }

  public FlywheelStates getState() {
    return currentState;
  }

  private FlywheelStates wantedState = FlywheelStates.OFF;
  private FlywheelStates currentState = FlywheelStates.OFF;
  private FlywheelStates previousState = FlywheelStates.OFF;

  /**
   * Check if flywheel is at setpoint using debounced tolerance checks. Updates both the fast
   * control mode debouncer and slower at-goal debouncer.
   *
   * @return true if velocity has been within tolerance for control mode debounce time
   */
  private boolean isAtSetpointVelocity() {
    double currentRPS = getLeaderVelocityRPS();
    boolean inTolerance = Math.abs(currentRPS - targetVelocityRPS) <= toleranceRPS.get();

    // Fast debouncer for control mode transitions (25ms default)
    boolean isAtSpeed = controlModeDebouncer.calculate(inTolerance);

    // Slower debouncer for external "ready to shoot" signal (200ms default)
    atGoal = atGoalDebouncer.calculate(inTolerance);

    return isAtSpeed;
  }

  private void updateState() {
    previousState = currentState;

    // State machine transitions
    switch (wantedState) {
      case SPINNING_UP:
        if (targetVelocityRPS == 0.0) {
          currentState = FlywheelStates.OFF;
        } else if (isAtSetpointVelocity()) {
          currentState = FlywheelStates.AT_SETPOINT;
        } else {
          currentState = FlywheelStates.SPINNING_UP;
        }
        break;

      case AT_SETPOINT:
        // Automatic transitions based on velocity
        if (targetVelocityRPS == 0.0) {
          currentState = FlywheelStates.OFF;
        } else if (!isAtSetpointVelocity()) {
          currentState = FlywheelStates.SPINNING_UP;
        } else {
          currentState = FlywheelStates.AT_SETPOINT;
        }
        break;

      case COAST:
        currentState = FlywheelStates.COAST;
        break;

      case OFF:
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
      case SPINNING_UP:
        // Duty cycle bang-bang for fast acceleration while spinning up
        currentControlMode = FlywheelControlType.DUTY_CYCLE_BANG_BANG;
        io.setVelocityBangBang(targetVelocityRPS);
        break;

      case AT_SETPOINT:
        // Torque current bang-bang to maintain speed with consistent torque
        currentControlMode = FlywheelControlType.TORQUE_CURRENT_BANG_BANG;
        io.setVelocityTorqueCurrentBangBang(targetVelocityRPS);
        break;

      case COAST:
        // Velocity PID control for coast mode
        currentControlMode = FlywheelControlType.VOLTAGE_VELOCITY;
        io.setVelocityPID(targetVelocityRPS);
        break;

      case OFF:
      default:
        currentControlMode = FlywheelControlType.STOP;
        io.stop();
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
    Logger.recordOutput("Subsystems/Flywheel/ControlMode", currentControlMode.toString());
    Logger.recordOutput("Subsystems/Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Subsystems/Flywheel/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Subsystems/Flywheel/AtSetpointVelocity", isAtSetpointVelocity());
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.stop();
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.stop());
  }

  public Command setVelocityCommand(double rps) {
    return this.runEnd(() -> setVelocityRPM(rps), () -> io.stop());
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(() -> setVelocityRPM(supplierVelocity.getAsDouble()), () -> io.stop());
  }
}
