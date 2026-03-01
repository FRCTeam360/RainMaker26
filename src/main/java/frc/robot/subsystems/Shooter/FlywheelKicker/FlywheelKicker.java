// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.FlywheelKicker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * FlywheelKicker subsystem that manages a bang-bang velocity controller for kicking notes into the
 * flywheel.
 *
 * <p>The kicker uses a state machine to coordinate spinup, setpoint holding, shot recovery, and
 * underspeed detection. State transitions follow this lifecycle:
 *
 * <pre>
 *   OFF → SPINNING_UP → AT_SETPOINT → RECOVERING → AT_SETPOINT (normal cycle)
 *                                      └→ UNDER_KICKING (sustained RPM drop)
 * </pre>
 *
 * <p>When controlled by the superstructure, the state machine runs every cycle in {@link
 * #periodic()}. Commands can bypass the state machine by setting the control state to {@link
 * ControlState#COMMAND}.
 */
public class FlywheelKicker extends SubsystemBase {
  // Constants
  private static final double KICKER_VELOCITY_RPM = 3000.0;
  private static final double TOLERANCE_RPM = 100.0;

  /** Debounce time for shot detection — filters noise from brief velocity dips. */
  private static final double BALL_FIRED_DEBOUNCE_SECONDS = 0.04;

  /** Debounce time for underspeed detection — sustained RPM drop signals too many rapid shots. */
  private static final double SUSTAINED_RPM_DROP_DEBOUNCE_SECONDS = 0.2;

  private long kickCount = 0;
  private static final double REVERSE_DUTY_CYCLE = -0.85;

  // IO fields
  private final FlywheelKickerIO io;
  private final FlywheelKickerIOInputsAutoLogged inputs = new FlywheelKickerIOInputsAutoLogged();

  // Enums
  public enum FlywheelKickerStates {
    IDLE,
    KICKING,
    REVERSING
  }

  public enum FlywheelKickerInternalStates {
    OFF,
    SPINNING_UP,
    AT_SETPOINT,
    RECOVERING,
    UNDER_KICKING
  }

  /**
   * Debouncer for shot detection. Prevents rapid spinup/hold switching when velocity briefly dips
   * below tolerance due to noise. A sustained drop past this debounce window indicates a note has
   * passed through the kicker. Uses {@link DebounceType#kFalling} so the "out of tolerance" signal
   * must persist before being accepted.
   */
  private final Debouncer ballFiredDebouncer =
      new Debouncer(BALL_FIRED_DEBOUNCE_SECONDS, DebounceType.kFalling);

  /**
   * Debouncer for underspeed detection. Detects when RPM has been below tolerance for too long,
   * indicating too many notes have passed through in rapid succession and the kicker can't recover
   * between shots. Uses {@link DebounceType#kFalling} so single-shot dips don't trigger it.
   */
  private final Debouncer underspeedDebouncer =
      new Debouncer(SUSTAINED_RPM_DROP_DEBOUNCE_SECONDS, DebounceType.kFalling);

  public enum FlywheelKickerInternalStates {
    OFF,
    SPINNING_UP,
    AT_SETPOINT,
    RECOVERING,
    UNDER_KICKING
  }

  /**
   * Debouncer for shot detection. Prevents rapid spinup/hold switching when velocity briefly dips
   * below tolerance due to noise. A sustained drop past this debounce window indicates a note has
   * passed through the kicker. Uses {@link DebounceType#kFalling} so the "out of tolerance" signal
   * must persist before being accepted.
   */
  private final Debouncer ballFiredDebouncer =
      new Debouncer(BALL_FIRED_DEBOUNCE_SECONDS, DebounceType.kFalling);

  /**
   * Debouncer for underspeed detection. Detects when RPM has been below tolerance for too long,
   * indicating too many notes have passed through in rapid succession and the kicker can't recover
   * between shots. Uses {@link DebounceType#kFalling} so single-shot dips don't trigger it.
   */
  private final Debouncer underspeedDebouncer =
      new Debouncer(SUSTAINED_RPM_DROP_DEBOUNCE_SECONDS, DebounceType.kFalling);

  // State variables
  private FlywheelKickerStates wantedState = FlywheelKickerStates.IDLE;
  private FlywheelKickerInternalStates currentState = FlywheelKickerInternalStates.OFF;
  private FlywheelKickerInternalStates previousState = FlywheelKickerInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new FlywheelKicker. */
  public FlywheelKicker(FlywheelKickerIO io) {
    this.io = io;
  }

  // State machine methods

  public FlywheelKickerInternalStates getState() {
    return currentState;
  }

  public void setWantedState(FlywheelKickerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private boolean atSetpoint() {
    boolean inTolerance = Math.abs(inputs.velocity - KICKER_VELOCITY_RPM) < TOLERANCE_RPM;
    return ballFiredDebouncer.calculate(inTolerance);
  }

  /**
   * Returns whether the kicker RPM has been below tolerance for too long, indicating too many notes
   * have passed through and the kicker can't recover between shots.
   *
   * @return true if the kicker is underspeed for a sustained period
   */
  private boolean isUnderspeed() {
    boolean inTolerance = Math.abs(inputs.velocity - KICKER_VELOCITY_RPM) < TOLERANCE_RPM;
    return !underspeedDebouncer.calculate(inTolerance);
  }

  /**
   * Updates the internal state based on the wanted state and current kicker velocity.
   *
   * <p>State transitions when kicking:
   *
   * <ul>
   *   <li>{@code UNDER_KICKING} — only from {@code RECOVERING}, when underspeed persists
   *   <li>{@code RECOVERING} — from {@code AT_SETPOINT} when a note is detected (velocity dip past
   *       debounce), or stays in {@code RECOVERING} while not yet back to setpoint
   *   <li>{@code AT_SETPOINT} — when velocity is within tolerance (from any active state)
   *   <li>{@code SPINNING_UP} — default active state during initial spinup
   * </ul>
   */
  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case KICKING:
        {
          boolean atBangBangSetpoint = atSetpoint();
          boolean underspeed = isUnderspeed();
          boolean wasAtSetpoint = previousState == FlywheelKickerInternalStates.AT_SETPOINT;
          boolean wasRecovering = previousState == FlywheelKickerInternalStates.RECOVERING;
          boolean ballFired = wasAtSetpoint && !atBangBangSetpoint;

          if (underspeed && wasRecovering) {
            currentState = FlywheelKickerInternalStates.UNDER_KICKING;
          } else if (ballFired) {
            kickCount++;
            currentState = FlywheelKickerInternalStates.RECOVERING;
          } else if (atBangBangSetpoint) {
            currentState = FlywheelKickerInternalStates.AT_SETPOINT;
          } else if (wasRecovering) {
            currentState = FlywheelKickerInternalStates.RECOVERING;
          } else {
            currentState = FlywheelKickerInternalStates.SPINNING_UP;
          }
          break;
        }
      case REVERSING:
        currentState = FlywheelKickerStates.REVERSING;
        break;
      case IDLE:
      default:
        currentState = FlywheelKickerInternalStates.OFF;
        break;
    }
  }

  /**
   * Applies motor outputs based on the current internal state.
   *
   * <ul>
   *   <li>{@code SPINNING_UP}, {@code RECOVERING}, {@code UNDER_KICKING} — aggressive spinup
   *       control for maximum acceleration
   *   <li>{@code AT_SETPOINT} — smooth hold control for consistent velocity with less overshoot
   *   <li>{@code OFF} — zero output
   * </ul>
   */
  private void applyState() {
    switch (currentState) {
      case SPINNING_UP:
      case RECOVERING:
      case UNDER_KICKING:
        setSpinupVelocityControl(KICKER_VELOCITY_RPM);
        break;
      case AT_SETPOINT:
        setHoldVelocityControl(KICKER_VELOCITY_RPM);
        break;
      case REVERSING:
        setDutyCycle(REVERSE_DUTY_CYCLE);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  // IO delegation methods

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
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
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double rpm) {
    return this.runEnd(() -> setVelocity(rpm), () -> setDutyCycle(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FlywheelKicker", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/FlywheelKicker/WantedState", wantedState);
    Logger.recordOutput("Subsystems/FlywheelKicker/CurrentState", currentState);
    Logger.recordOutput("Subsystems/FlywheelKicker/PreviousState", previousState);
    Logger.recordOutput("Subsystems/FlywheelKicker/ControlState", controlState);
    Logger.recordOutput("Subsystems/FlywheelKicker/KickCount", kickCount);
  }
}
