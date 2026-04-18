// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachineSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends StateMachineSubsystem<IndexerIOInputsAutoLogged, IndexerIO> {
  // Constants
  private static final double INDEXER_VELOCITY_RPM = 3000.0;
  private static final double INDEXER_DUTY_CYCLE = 0.80;
  private static final double INTAKING_ASSIST_DUTY_CYCLE = -0.15;
  private static final double REVERSING_DUTY_CYCLE = -0.35;

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

  // Constructor

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    super(io, new IndexerIOInputsAutoLogged());
  }

  // State machine methods

  public IndexerStates getState() {
    return currentState;
  }

  public void setWantedState(IndexerStates state) {
    wantedState = state;
  }

  @Override
  protected void updateState() {
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

  @Override
  protected void applyState() {
    switch (currentState) {
      case ASSIST_INTAKING:
        setDutyCycle(INTAKING_ASSIST_DUTY_CYCLE);
        break;
      case INDEXING:
        setVelocity(INDEXER_VELOCITY_RPM);
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
  protected void logOutputs() {
    Logger.recordOutput("Superstructure/Subsystems/Indexer/WantedState", wantedState);
    Logger.recordOutput("Superstructure/Subsystems/Indexer/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Subsystems/Indexer/PreviousState", previousState);
    Logger.recordOutput("Superstructure/Subsystems/Indexer/ControlState", controlState);
  }
}
