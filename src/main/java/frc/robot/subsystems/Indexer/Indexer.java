// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  // Constants
  private static final double INDEXER_DUTY_CYCLE = 0.85;
  private static final double INTAKING_ASSIST_DUTY_CYCLE = -0.15;
  private static final double REVERSING_DUTY_CYCLE = -0.35;

  // IO fields
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  // Enums
  public enum IndexerStates {
    OFF,
    ASSIST_INTAKING,
    INDEXING,
    REVERSING
  }

  // State variables
  private IndexerStates wantedState = IndexerStates.OFF;
  private IndexerStates currentState = IndexerStates.OFF;
  private IndexerStates previousState = IndexerStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  // State machine methods

  public IndexerStates getState() {
    return currentState;
  }

  public void setWantedState(IndexerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case ASSIST_INTAKING:
        currentState = IndexerStates.ASSIST_INTAKING;
        break;

      case INDEXING:
        currentState = IndexerStates.INDEXING;
        break;
      case REVERSING:
        currentState = IndexerStates.REVERSING;
        break;
      case OFF:
        currentState = IndexerStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case ASSIST_INTAKING:
        setDutyCycle(INTAKING_ASSIST_DUTY_CYCLE);
        break;
      case INDEXING:
        setDutyCycle(INDEXER_DUTY_CYCLE);
        break;
      case REVERSING:
        setDutyCycle(REVERSING_DUTY_CYCLE);
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

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/Indexer/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Indexer/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Indexer/PreviousState", previousState);
    Logger.recordOutput("Subsystems/Indexer/ControlState", controlState);
  }
}
