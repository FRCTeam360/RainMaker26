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

/**
 * Flywheel subsystem that manages a bang-bang velocity controller for shooting.
 *
 * <p>The flywheel uses a state machine to coordinate spinup, setpoint holding, shot recovery, and
 * underspeed detection. State transitions follow this lifecycle:
 *
 * <pre>
 *   OFF → SPINNING_UP → AT_SETPOINT → RECOVERING → AT_SETPOINT (normal cycle)
 *                                      └→ UNDER_SHOOTING (sustained RPM drop)
 * </pre>
 *
 * <p>When controlled by the superstructure, the state machine runs every cycle in {@link
 * #periodic()}. Commands can bypass the state machine by setting the control state to {@link
 * ControlState#COMMAND}.
 */
public class Flywheel extends SubsystemBase {
  /** Velocity tolerance for bang-bang setpoint detection. */
  private static final double TOLERANCE_RPM = 150.0;

  /** Debounce time for shot detection — filters noise from brief velocity dips. */
  private static final double BALL_FIRED_DEBOUNCE_SECONDS = 0.04;

  /** Debounce time for underspeed detection — sustained RPM drop signals too many rapid shots. */
  private static final double SUSTAINED_RPM_DROP_DEBOUNCE_SECOND = 0.4;

  private long launchCount = 0;

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private DoubleSupplier shootVelocitySupplier = () -> 0.0;

  /** Wanted states set by the superstructure to command the flywheel. */
  public enum FlywheelWantedStates {
    IDLE,
    SHOOTING,
    COASTING
  }

  /**
   * Internal states representing the flywheel's current phase in the bang-bang control lifecycle.
   *
   * <ul>
   *   <li>{@link #OFF} — motors stopped, no control applied
   *   <li>{@link #SPINNING_UP} — initial spinup using duty-cycle bang-bang (max acceleration)
   *   <li>{@link #AT_SETPOINT} — velocity within tolerance, holding with torque-current bang-bang
   *   <li>{@link #RECOVERING} — re-spinning after a ball fired; can still transition to firing in
   *       the shooter state machine during brief dips
   *   <li>{@link #UNDER_SHOOTING} — sustained RPM drop after reaching setpoint, indicating the
   *       flywheel can't recover between rapid shots; only reachable from {@link #RECOVERING}
   * </ul>
   */
  public enum FlywheelInternalStates {
    OFF,
    SPINNING_UP,
    AT_SETPOINT,
    RECOVERING,
    UNDER_SHOOTING,
    COAST
  }

  /**
   * Debouncer for shot detection. Prevents rapid spinup/hold switching when velocity briefly dips
   * below tolerance due to noise. A sustained drop past this debounce window indicates a ball has
   * passed through the flywheel. Uses {@link DebounceType#kFalling} so the "out of tolerance"
   * signal must persist before being accepted.
   */
  private final Debouncer ballFiredDebouncer =
      new Debouncer(BALL_FIRED_DEBOUNCE_SECONDS, DebounceType.kFalling);

  /**
   * Debouncer for underspeed detection. Detects when RPM has been below tolerance for too long,
   * indicating too many balls have passed through in rapid succession and the flywheel can't
   * recover between shots. Uses {@link DebounceType#kFalling} so single-shot dips don't trigger it.
   */
  private final Debouncer underspeedDebouncer =
      new Debouncer(SUSTAINED_RPM_DROP_DEBOUNCE_SECOND, DebounceType.kFalling);

  // State variables
  private FlywheelWantedStates wantedState = FlywheelWantedStates.IDLE;
  private FlywheelInternalStates currentState = FlywheelInternalStates.OFF;
  private FlywheelInternalStates previousState = FlywheelInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  // State machine methods

  /** Returns the current internal state of the flywheel. */
  public FlywheelInternalStates getState() {
    return currentState;
  }

  /**
   * Sets the wanted state of the flywheel, used by the superstructure.
   *
   * @param state the desired flywheel state
   */
  public void setWantedState(FlywheelWantedStates state) {
    wantedState = state;
  }

  /**
   * Sets whether the flywheel is controlled by the superstructure or by commands directly.
   *
   * @param controlState the control state to set
   */
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

  /**
   * Updates the internal state based on the wanted state and current flywheel velocity.
   *
   * <p>State transitions when shooting:
   *
   * <ul>
   *   <li>{@code UNDER_SHOOTING} — only from {@code RECOVERING}, when underspeed persists
   *   <li>{@code RECOVERING} — from {@code AT_SETPOINT} when a ball is detected (velocity dip past
   *       debounce), or stays in {@code RECOVERING} while not yet back to setpoint
   *   <li>{@code AT_SETPOINT} — when velocity is within tolerance (from any active state)
   *   <li>{@code SPINNING_UP} — default active state during initial spinup
   * </ul>
   */
  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING:
        {
          double targetRPM = shootVelocitySupplier.getAsDouble();
          boolean atBangBangSetpoint = atSetpoint(targetRPM);
          boolean underspeed = isUnderspeed(targetRPM);
          boolean wasAtSetpoint = previousState == FlywheelInternalStates.AT_SETPOINT;
          boolean wasRecovering = previousState == FlywheelInternalStates.RECOVERING;
          boolean ballFired = wasAtSetpoint && !atBangBangSetpoint;

          if (underspeed && wasRecovering) {
            currentState = FlywheelInternalStates.UNDER_SHOOTING;
          } else if (ballFired) {
            launchCount++;
            currentState = FlywheelInternalStates.RECOVERING;
          } else if (atBangBangSetpoint) {
            currentState = FlywheelInternalStates.AT_SETPOINT;
          } else if (wasRecovering) {
            currentState = FlywheelInternalStates.RECOVERING;
          } else {
            currentState = FlywheelInternalStates.SPINNING_UP;
          }
          break;
        }
      case COASTING:
        currentState = FlywheelInternalStates.COAST;
        break;
      case IDLE:
      default:
        currentState = FlywheelInternalStates.OFF;
        break;
    }
  }

  /**
   * Applies motor outputs based on the current internal state.
   *
   * <ul>
   *   <li>{@code SPINNING_UP}, {@code RECOVERING}, {@code UNDER_SHOOTING} — duty-cycle bang-bang
   *       for maximum acceleration
   *   <li>{@code AT_SETPOINT} — torque-current bang-bang for consistent hold with less overshoot
   *   <li>{@code OFF} — zero output
   * </ul>
   */
  private void applyState() {
    switch (currentState) {
      case SPINNING_UP:
      case RECOVERING:
      case UNDER_SHOOTING:
        setSpinupVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case AT_SETPOINT:
        setHoldVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case COAST:
        setCoastVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case OFF:
      default:
        setDutyCycle(0.0);
        break;
    }
  }

  /**
   * Sets the flywheel to coast velocity control (for gentle deceleration or low-power holding).
   *
   * @param rpm the target velocity in RPM
   */
  public void setCoastVelocityControl(double rpm) {
    io.setCoastVelocityControl(rpm);
  }

  /**
   * Returns whether the flywheel velocity is within tolerance of the target, debounced to filter
   * noise. A falling-edge debounce ensures brief dips below tolerance don't immediately register as
   * "not at setpoint."
   *
   * @param targetRPM the target velocity in RPM
   * @return true if the flywheel has been at setpoint for the debounce duration
   */
  private boolean atSetpoint(double targetRPM) {
    if (inputs.velocities.length > 0) {
      boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
      return ballFiredDebouncer.calculate(inTolerance);
    }
    return false;
  }

  /**
   * Returns whether the flywheel RPM has been below tolerance for too long, indicating too many
   * balls have passed through and the flywheel can't recover between shots.
   *
   * @param targetRPM the target velocity in RPM
   * @return true if the flywheel is underspeed for a sustained period
   */
  private boolean isUnderspeed(double targetRPM) {
    if (inputs.velocities.length > 0) {
      boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
      return !underspeedDebouncer.calculate(inTolerance);
    }
    return true;
  }

  /**
   * Sets the flywheel duty cycle directly (open loop).
   *
   * @param duty the duty cycle to apply, in the range [-1.0, 1.0]
   */
  private void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  /**
   * Sets the flywheel to spinup velocity control (duty-cycle bang-bang for max acceleration).
   *
   * @param rpm the target velocity in RPM
   */
  public void setSpinupVelocityControl(double rpm) {
    io.setSpinupVelocityControl(rpm);
  }

  /**
   * Sets the flywheel to hold velocity control (torque-current bang-bang for consistent hold).
   *
   * @param rpm the target velocity in RPM
   */
  public void setHoldVelocityControl(double rpm) {
    io.setHoldVelocityControl(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

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
