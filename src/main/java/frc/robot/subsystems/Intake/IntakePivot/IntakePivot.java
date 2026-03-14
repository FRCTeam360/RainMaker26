// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

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
  private static final double HIGH_AGITATED_POSITION_DEGREES = 45.0;
  private static final double LOW_AGITATED_POSITION_DEGREES = 80.0;
  private static final double STACK_FUEL_POSITION_DEGREES = 20.0;
  private static final double TOLERANCE_DEGREES = 2.0;
  // IO fields
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public enum IntakePivotWantedStates {
    IDLE,
    STOWED,
    DEPLOYED,
    AGITATE_HOPPER,
    STACK_FUEL
  }

  public enum IntakePivotInternalStates {
    IDLE,
    MOVING_TO_SETPOINT,
    AT_SETPOINT,
    SWITCHING_AGITATE_TARGET_HIGH,
    SWITCHING_AGITATE_TARGET_LOW
  }

  // State variables
  private IntakePivotWantedStates wantedState = IntakePivotWantedStates.IDLE;
  private IntakePivotInternalStates currentState = IntakePivotInternalStates.IDLE;
  private IntakePivotInternalStates previousState = IntakePivotInternalStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;
  // For agitation cycle
  private boolean agitateTargetHigh = true;

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
      case AGITATE_HOPPER:
        {
          double target =
              agitateTargetHigh ? HIGH_AGITATED_POSITION_DEGREES : LOW_AGITATED_POSITION_DEGREES;
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
      case STACK_FUEL:
      default:
        currentState = IntakePivotInternalStates.IDLE;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case MOVING_TO_SETPOINT:
      case AT_SETPOINT:
      case SWITCHING_AGITATE_TARGET_HIGH:
      case SWITCHING_AGITATE_TARGET_LOW:
        setPosition(getTargetPosition());
        break;
      case IDLE:
      default:
        stop();
    }
  }

  private double getTargetPosition() {
    switch (wantedState) {
      case AGITATE_HOPPER:
        return agitateTargetHigh ? HIGH_AGITATED_POSITION_DEGREES : LOW_AGITATED_POSITION_DEGREES;
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

  public boolean atSetpoint(double setpoint) {
    return Math.abs(inputs.position - setpoint) < TOLERANCE_DEGREES;
  }

  // No longer needed: shouldLowToHigh/shouldHighToLow

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
