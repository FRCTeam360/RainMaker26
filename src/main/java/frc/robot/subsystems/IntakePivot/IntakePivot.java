// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  // Constants
  private static final double STOWED_POSITION_DEGREES = 0.0;
  private static final double DEPLOYED_POSITION_DEGREES = 93.0;
  private static final double HIGH_AGITATED_POSITION = 60.0;
  private static final double LOW_AGITATED_POISTION = 30.0;

  // IO fields
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public enum IntakeWantedStates {
    IDLE,
    STOWED,
    DEPLOYED,
  }

  public enum IntakePivotInternalStates {
    IDLE,
    MOVING_TO_SETPOINT,
    AT_SETPOINT
  }

  // State variables
  private IntakeWantedStates wantedState = IntakeWantedStates.IDLE;
  private IntakePivotInternalStates currentState = IntakePivotInternalStates.IDLE;
  private IntakePivotInternalStates previousState = IntakePivotInternalStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new IntakePivot. */
  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  // State machine methods

  public IntakePivotInternalStates getState() {
    return currentState;
  }

  public void setWantedState(IntakeWantedStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case STOWED:
        currentState = IntakeWantedStates.STOWED;
        break;
      case DEPLOYED:
        currentState = IntakeWantedStates.DEPLOYED;
        break;
      default:
        currentState = IntakeWantedStates.IDLE;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case DEPLOYED:
        setPosition(DEPLOYED_POSITION_DEGREES);
        break;
      case STOWED:
        setPosition(STOWED_POSITION_DEGREES);
        break;
      case IDLE:
      default:
        stop();
    }
  }

  // IO delegation methods

  public void setPosition(double value) {
    io.setPosition(value);
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
  }
}
