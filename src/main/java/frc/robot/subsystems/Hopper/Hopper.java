// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public enum HopperStates {
    OFF,
    INTAKING,
  }

  private HopperStates wantedState = HopperStates.OFF;
  private HopperStates currentState = HopperStates.OFF;
  private HopperStates previousState = HopperStates.OFF;

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = HopperStates.INTAKING;
        break;
      case OFF:
        currentState = HopperStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        setDutyCycle(0.2);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  /** Creates a new Hopper. */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  public void setWantedState(HopperStates state) {
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
    Logger.processInputs("Hopper", inputs);
    Logger.recordOutput("Subsystems/Hopper/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Hopper/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Hopper/PreviousState", previousState.toString());
  }
}
