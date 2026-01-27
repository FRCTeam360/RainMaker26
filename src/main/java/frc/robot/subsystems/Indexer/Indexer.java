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
    COLLECTING_FUEL
  }

  private IndexerStates wantedState = IndexerStates.OFF;
  private IndexerStates currentState = IndexerStates.OFF;
  private IndexerStates previousState = IndexerStates.OFF;

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case COLLECTING_FUEL:
        currentState = IndexerStates.COLLECTING_FUEL;
        break;

      case OFF:
      default:
        currentState = IndexerStates.OFF;
        break;
    }
  }
  private void applyState() {
    switch (currentState) {
      case COLLECTING_FUEL:
        setDutyCycle(-0.65);
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
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  @Override
  public void periodic() {
     updateState();
    applyState();

    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
     Logger.recordOutput("Subsystems/Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Intake/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Intake/PreviousState", previousState.toString());
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }
}
