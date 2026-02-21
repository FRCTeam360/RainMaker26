// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class HopperRoller extends SubsystemBase {
  private final HopperRollerIO io;
  private final HopperRollerIOInputsAutoLogged inputs = new HopperRollerIOInputsAutoLogged();

  public enum HopperRollerStates {
    OFF,
    ROLLING
  }

  private HopperRollerStates wantedState = HopperRollerStates.OFF;
  private HopperRollerStates currentState = HopperRollerStates.OFF;
  private HopperRollerStates previousState = HopperRollerStates.OFF;

  private static final double ROLLER_DUTY_CYCLE = 1.0;

  public HopperRollerStates getState() {
    return currentState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case ROLLING:
        currentState = HopperRollerStates.ROLLING;
        break;
      case OFF:
      default:
        currentState = HopperRollerStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case ROLLING:
        setDutyCycle(ROLLER_DUTY_CYCLE);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  public HopperRoller(HopperRollerIO io) {
    this.io = io;
  }

  public void setWantedState(HopperRollerStates state) {
    wantedState = state;
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
    Logger.processInputs("HopperRoller", inputs);

    updateState();
    applyState();
    Logger.recordOutput("Subsystems/HopperRoller/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/HopperRoller/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/HopperRoller/PreviousState", previousState.toString());
  }
}
