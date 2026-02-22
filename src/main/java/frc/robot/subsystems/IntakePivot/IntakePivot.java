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
  public final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  public final IntakePivotIO io;
  private final IntakePivotVisualizer visualizer;
  private static final double STOWED_POSITION = 0.0;
  private static final double DEPLOYED_POSITION = 90.0;

  public enum IntakePivotStates {
    OFF,
    STOWED,
    DEPLOYED,
  }

  private IntakePivotStates wantedState = IntakePivotStates.OFF;
  private IntakePivotStates previousState = IntakePivotStates.OFF;
  private IntakePivotStates currentState = IntakePivotStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case STOWED:
        currentState = IntakePivotStates.STOWED;
        break;
      case DEPLOYED:
        currentState = IntakePivotStates.DEPLOYED;
        break;
      default:
        currentState = IntakePivotStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case DEPLOYED:
        setPosition(DEPLOYED_POSITION);
        break;
      case STOWED:
        setPosition(STOWED_POSITION);
        break;
      case OFF:
      default:
        setPosition(0.0);
    }
  }

  /** Creates a new IntakePivot. */
  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    // Initialize visualizer with arm length in meters (30 inches = 0.762 m)
    this.visualizer = new IntakePivotVisualizer(0.762);
  }

  public IntakePivotStates getState() {
    return currentState;
  }

  public void setWantedState(IntakePivotStates state) {
    wantedState = state;
  }

  public void setPosition(double value) {
    io.setPosition(value);
  }

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }

  public Command setPosition(DoubleSupplier positionSupplier) {
    return this.runEnd(() -> this.setPosition(positionSupplier.getAsDouble()), () -> this.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/IntakePivot/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/IntakePivot/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/IntakePivot/PreviousState", previousState.toString());
    // Update visualization with current arm angle (convert rotations to radians)
    visualizer.update(inputs.position * 2.0 * Math.PI);
  }
}
