// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake.IntakeStates;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public enum ClimberStates {
    OFF,
    ROLLING
  }

  private ClimberStates wantedState = ClimberStates.OFF;
  private ClimberStates currentState = ClimberStates.OFF;
  private ClimberStates previousState = ClimberStates.OFF;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public ClimberStates getState() {
    return currentState;
  }

  public void setWantedState(ClimberStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case ROLLING:
        currentState = ClimberStates.ROLLING;
        break;
      case OFF:
      default:
        currentState = ClimberStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case ROLLING:
        climbing();
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  public void setLeftDutyCycle(double dutyCycle) {
    io.setLeftDutyCycle(dutyCycle);
  }

  public void setRightDutyCycle(double dutyCycle) {
    io.setRightDutyCycle(dutyCycle);
  }

  public void climbing() {
    io.setLeftPosition(0);
    io.setRightPosition(0);
    // TODO add actual position and climbing sequence
  }

  public void stop() {
    io.setLeftDutyCycle(0.0);
    io.setRightDutyCycle(0.0);
  }

  public boolean leftAboveMinHeight() {
    return io.leftAboveMinHeight();
  }

  public boolean rightAboveMinHeight(){
    return io.rightAboveMinHeight();
  }

  public void setLeftPosition(double position) {
    io.setLeftPosition(position);
  }

  public void setRightPosition(double position) {
    io.setRightPosition(position);
  }

  public void zeroBoth() {
    io.zeroBoth();
  }


  public void updatePIDF(double P, double I, double D, double F) {
    io.updatePIDF(P, I, D, F);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Subsystems/Climber/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Climber/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Climber/PreviousState", previousState.toString());
  }
}
