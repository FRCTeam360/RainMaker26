// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake.IntakeStates;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public enum IndexerStates {
    OFF,
    INTAKING,
  }

  private IndexerStates wantedState = IndexerStates.OFF;
  private IndexerStates currentState = IndexerStates.OFF;
  private IndexerStates previousState = IndexerStates.OFF;

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = IndexerStates.INTAKING;
        break;
      case OFF:
        currentState = IndexerStates.OFF;
        break;
    }
  }
  private void applyState() {
    switch (currentState) {
      case INTAKING:
        setDutyCycle(0.5);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }
 public void setWantedState(IndexerStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Subsystems/Indexer/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Indexer/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Indexer/PreviousState", previousState.toString());
  }
}
