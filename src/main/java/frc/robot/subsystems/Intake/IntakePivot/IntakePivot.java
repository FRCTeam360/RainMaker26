// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  // Constants
  private static final double STOWED_POSITION_DEGREES = 0.0;
  private static final double DEPLOYED_POSITION_DEGREES = 97.0;
  private static final double AGITATE_LOW_UPPER_POSITION_DEGREES = 20.0;
  private static final double AGITATE_LOW_LOWER_POSITION_DEGREES = 0.0;
  private static final double AGITATE_HIGH_UPPER_POSITION_DEGREES = 75.0;
  private static final double AGITATE_HIGH_LOWER_POSITION_DEGREES = 55.0;
  private static final double TOLERANCE_DEGREES = 2.0;
  private static final double SOFT_LIMIT_PROXIMITY_DEGREES = 5.0;
  private static final double FORWARD_SOFT_LIMIT_DEGREES = 97.0;
  private static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0;
  // Progressive agitate constants
  private static final double PROGRESSIVE_AGITATE_DURATION_SECONDS = 6.0;
  private static final double PROGRESSIVE_AGITATE_DIP_DEGREES = 10.0;
  private static final double PROGRESSIVE_AGITATE_OSCILLATIONS_PER_SECOND = 2.0;
  // Stall detection constants
  private static final double STALL_CURRENT_THRESHOLD_AMPS = 40.0;
  private static final double STALL_VELOCITY_THRESHOLD_DPS = 5.0;
  private static final double STALL_BACKOFF_DEGREES = 10.0;
  // IO fields
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public enum IntakePivotWantedStates {
    IDLE,
    STOWED,
    DEPLOYED,
    AGITATE_HOPPER_LOW,
    AGITATE_HOPPER_HIGH,
    AGITATE_PROGRESSIVE
  }

  public enum IntakePivotInternalStates {
    IDLE,
    MOVING_TO_SETPOINT,
    AT_SETPOINT,
    SWITCHING_AGITATE_TARGET_HIGH,
    SWITCHING_AGITATE_TARGET_LOW,
    PROGRESSIVE_AGITATING,
    PROGRESSIVE_COMPLETE,
    STALLING
  }

  // State variables
  private IntakePivotWantedStates wantedState = IntakePivotWantedStates.IDLE;
  private IntakePivotInternalStates currentState = IntakePivotInternalStates.IDLE;
  private IntakePivotInternalStates previousState = IntakePivotInternalStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;
  // For agitation cycle
  private boolean agitateTargetHigh = true;
  // For progressive agitate
  private final Timer progressiveTimer = new Timer();
  private boolean progressiveTimerStarted = false;
  // Stall backoff — accumulated degrees to back off toward deployed
  private double stallBackoffDegrees = 0.0;

  // Constructor

  /** Creates a new IntakePivot. */
  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  // State machine methods

  public IntakePivotInternalStates getState() {
    return currentState;
  }

  public void setWantedState(IntakePivotWantedStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case STOWED:
        if (atSetpoint(STOWED_POSITION_DEGREES)) {
          currentState = IntakePivotInternalStates.AT_SETPOINT;
        } else {
          currentState = IntakePivotInternalStates.MOVING_TO_SETPOINT;
        }
        break;
      case DEPLOYED:
        currentState =
            atSetpoint(DEPLOYED_POSITION_DEGREES)
                ? IntakePivotInternalStates.AT_SETPOINT
                : IntakePivotInternalStates.MOVING_TO_SETPOINT;
        break;
      case AGITATE_HOPPER_LOW:
      case AGITATE_HOPPER_HIGH:
        {
          double upperTarget = getAgitateUpperPosition();
          double lowerTarget = getAgitateLowerPosition();
          double target = agitateTargetHigh ? upperTarget : lowerTarget;
          boolean atTarget = atSetpoint(target);
          if (previousState == IntakePivotInternalStates.MOVING_TO_SETPOINT && atTarget) {
            currentState =
                agitateTargetHigh
                    ? IntakePivotInternalStates.SWITCHING_AGITATE_TARGET_LOW
                    : IntakePivotInternalStates.SWITCHING_AGITATE_TARGET_HIGH;
          } else if (currentState == IntakePivotInternalStates.SWITCHING_AGITATE_TARGET_HIGH) {
            agitateTargetHigh = true;
            currentState = IntakePivotInternalStates.AT_SETPOINT;
          } else if (currentState == IntakePivotInternalStates.SWITCHING_AGITATE_TARGET_LOW) {
            agitateTargetHigh = false;
            currentState = IntakePivotInternalStates.AT_SETPOINT;
          } else {
            currentState =
                atTarget
                    ? IntakePivotInternalStates.AT_SETPOINT
                    : IntakePivotInternalStates.MOVING_TO_SETPOINT;
          }
          break;
        }
      case AGITATE_PROGRESSIVE:
        {
          if (!progressiveTimerStarted) {
            progressiveTimer.restart();
            progressiveTimerStarted = true;
          }
          // if (isStalling()) {
          //   stallBackoffDegrees += STALL_BACKOFF_DEGREES;
          //   currentState = IntakePivotInternalStates.STALLING;
          //   break;
          // }
          // stallBackoffDegrees = 0.0;
          double elapsedSeconds = progressiveTimer.get();
          double progress = Math.min(elapsedSeconds / PROGRESSIVE_AGITATE_DURATION_SECONDS, 1.0);
          double basePositionDegrees =
              DEPLOYED_POSITION_DEGREES
                  + (STOWED_POSITION_DEGREES - DEPLOYED_POSITION_DEGREES) * progress;
          if (elapsedSeconds >= PROGRESSIVE_AGITATE_DURATION_SECONDS
              || basePositionDegrees <= STOWED_POSITION_DEGREES + PROGRESSIVE_AGITATE_DIP_DEGREES) {
            progressiveTimerStarted = false;
            currentState = IntakePivotInternalStates.PROGRESSIVE_COMPLETE;
          } else {
            currentState = IntakePivotInternalStates.PROGRESSIVE_AGITATING;
          }
          break;
        }
      default:
        stallBackoffDegrees = 0.0;
        currentState = IntakePivotInternalStates.IDLE;
        break;
    }
  }

  private boolean isStalling() {
    return inputs.statorCurrent > STALL_CURRENT_THRESHOLD_AMPS
        && Math.abs(inputs.velocity) < STALL_VELOCITY_THRESHOLD_DPS;
  }

  private boolean isNearSoftLimit(double targetDegrees) {
    return targetDegrees <= REVERSE_SOFT_LIMIT_DEGREES + SOFT_LIMIT_PROXIMITY_DEGREES
        || targetDegrees >= FORWARD_SOFT_LIMIT_DEGREES - SOFT_LIMIT_PROXIMITY_DEGREES;
  }

  private void applyState() {
    switch (currentState) {
      case MOVING_TO_SETPOINT:
      case AT_SETPOINT:
      case SWITCHING_AGITATE_TARGET_HIGH:
      case SWITCHING_AGITATE_TARGET_LOW:
      case PROGRESSIVE_AGITATING:
      case STALLING:
        double target = getTargetPosition() + stallBackoffDegrees;
        target = Math.max(STOWED_POSITION_DEGREES, Math.min(target, DEPLOYED_POSITION_DEGREES));
        if (isNearSoftLimit(target)) {
          setPositionSmooth(target);
        } else {
          setPositionAggressive(target);
        }
        break;
      case PROGRESSIVE_COMPLETE:
        setPositionSmooth(STOWED_POSITION_DEGREES);
        break;
      case IDLE:
      default:
        stallBackoffDegrees = 0.0;
        stop();
    }
  }

  private double getAgitateUpperPosition() {
    return wantedState == IntakePivotWantedStates.AGITATE_HOPPER_HIGH
        ? AGITATE_HIGH_UPPER_POSITION_DEGREES
        : AGITATE_LOW_UPPER_POSITION_DEGREES;
  }

  private double getAgitateLowerPosition() {
    return wantedState == IntakePivotWantedStates.AGITATE_HOPPER_HIGH
        ? AGITATE_HIGH_LOWER_POSITION_DEGREES
        : AGITATE_LOW_LOWER_POSITION_DEGREES;
  }

  /**
   * Computes the progressive agitate setpoint as a function of elapsed time. Linearly interpolates
   * from deployed to stowed over the configured duration, with a square wave dip toward deployed
   * superimposed for maximum aggression.
   */
  private double getProgressiveAgitatePosition() {
    double elapsedSeconds = progressiveTimer.get();
    double progress = Math.min(elapsedSeconds / PROGRESSIVE_AGITATE_DURATION_SECONDS, 1.0);
    // Linear interpolation from deployed toward stowed
    double basePositionDegrees =
        DEPLOYED_POSITION_DEGREES
            + (STOWED_POSITION_DEGREES - DEPLOYED_POSITION_DEGREES) * progress;
    // Square wave: snap between base and base + dip (toward deployed, never below stowed)
    double halfPeriodSeconds = 1.0 / (2.0 * PROGRESSIVE_AGITATE_OSCILLATIONS_PER_SECOND);
    boolean dipped = (elapsedSeconds % (2.0 * halfPeriodSeconds)) >= halfPeriodSeconds;
    double oscillation = dipped ? PROGRESSIVE_AGITATE_DIP_DEGREES : 0.0;
    double position = basePositionDegrees + oscillation;
    return Math.max(STOWED_POSITION_DEGREES, Math.min(position, DEPLOYED_POSITION_DEGREES));
  }

  private double getTargetPosition() {
    switch (wantedState) {
      case AGITATE_HOPPER_LOW:
      case AGITATE_HOPPER_HIGH:
        return agitateTargetHigh ? getAgitateUpperPosition() : getAgitateLowerPosition();
      case AGITATE_PROGRESSIVE:
        return getProgressiveAgitatePosition();
      case DEPLOYED:
        return DEPLOYED_POSITION_DEGREES;
      case STOWED:
        return STOWED_POSITION_DEGREES;
      default:
        return STOWED_POSITION_DEGREES;
    }
  }

  // IO delegation methods

  public void setPosition(double value) {
    io.setPosition(value);
  }

  public void setPositionSmooth(double value) {
    io.setPositionSmooth(value);
  }

  public void setPositionAggressive(double value) {
    io.setPositionAggressive(value);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(inputs.position - setpoint) < TOLERANCE_DEGREES;
  }

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }

  public Command setPositionCommand(DoubleSupplier positionSupplier) {
    return this.runOnce(() -> this.setPosition(positionSupplier.getAsDouble()));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/IntakePivot/WantedState", wantedState);
    Logger.recordOutput("Subsystems/IntakePivot/CurrentState", currentState);
    Logger.recordOutput("Subsystems/IntakePivot/PreviousState", previousState);
    Logger.recordOutput("Subsystems/IntakePivot/ControlState", controlState);
    SmartDashboard.putString(
        "Subsystems/IntakePivot/CurrentIntakePivotState", currentState.toString());
  }
}
