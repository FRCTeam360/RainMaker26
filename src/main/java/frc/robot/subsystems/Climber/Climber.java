// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private static final double MIN_HEIGHT_THRESHOLD_METERS = 0;
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public enum ClimberStates {
    IDLE,
    EXTENDING,
    RETRACTING,
    LOCKED
  }

  private ClimberStates wantedState = ClimberStates.IDLE;
  private ClimberStates currentState = ClimberStates.IDLE;
  private ClimberStates previousState = ClimberStates.IDLE;

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
      case EXTENDING:
        currentState = ClimberStates.EXTENDING;
        break;
      case RETRACTING:
        currentState = ClimberStates.RETRACTING;
        break;
      case LOCKED:
        currentState = ClimberStates.LOCKED;
        break;
      case IDLE:
      default:
        currentState = ClimberStates.IDLE;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case EXTENDING:
        break;
      case RETRACTING:
        climbing();
        break;
      case LOCKED:
        break;
      case IDLE:

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

  public Command setRightDutyCycleCommand(double dutyCycle) {
    return this.runEnd(() -> this.setRightDutyCycle(dutyCycle), () -> this.setRightDutyCycle(0.0));
  }

  public Command setLeftDutyCycleCommand(double dutyCycle) {
    return this.runEnd(() -> this.setLeftDutyCycle(dutyCycle), () -> this.setLeftDutyCycle(0.0));
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

  public boolean rightAboveMinHeight() {
    return inputs.climberRightPosition > MIN_HEIGHT_THRESHOLD_METERS;
  }

  public boolean leftAboveMinHeight() {
    return inputs.climberLeftPosition > MIN_HEIGHT_THRESHOLD_METERS;
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Subsystems/Climber/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Climber/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Climber/PreviousState", previousState.toString());
  }
}
