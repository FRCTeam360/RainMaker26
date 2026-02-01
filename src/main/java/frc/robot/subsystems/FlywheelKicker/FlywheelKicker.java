// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FlywheelKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelKicker extends SubsystemBase {
  private final FlywheelKickerIO io;
  private final FlywheelKickerIOInputsAutoLogged inputs = new FlywheelKickerIOInputsAutoLogged();
public enum FlywheelKickerStates {
    OFF,
    INTAKING
  }
  private FlywheelKickerStates wantedState = FlywheelKickerStates.OFF;
  private FlywheelKickerStates currentState = FlywheelKickerStates.OFF;
  private FlywheelKickerStates previousState = FlywheelKickerStates.OFF;

   private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = FlywheelKickerStates.INTAKING;
        break;
      case OFF:
        currentState =  FlywheelKickerStates.OFF;
        break;
    }

  }
  private void applyState() {
    switch (currentState) {
      case INTAKING:
        setDutyCycle(1.0);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }
  public void setWantedState(FlywheelKickerStates state) {
    wantedState = state;
    updateState();
    applyState();
  }
  /** Creates a new FlywheelKicker. */
  public FlywheelKicker(FlywheelKickerIO io) {
    this.io = io;
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
    // updateState();
    // applyState();
    io.updateInputs(inputs);
    Logger.processInputs("FlywheelKicker", inputs);
    Logger.recordOutput("Subsystems/FlywheelKicker/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/FlywheelKicker/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/FlywheelKicker/PreviousState", previousState.toString());
  }
}
