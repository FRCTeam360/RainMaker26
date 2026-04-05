// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import org.littletonrobotics.junction.Logger;

public class HopperRoller extends SubsystemBase {
  // Constants
  private static final double ROLLER_VELOCITY_RPM = 4000.0;
  private static final double ROLLER_DUTY_CYCLE = 0.80;
  private static final double PREVENT_JAM_DUTY_CYCLE = -0.04;
  private static final double UNJAMMING_DUTY_CYCLE = -0.95;
  private static final double REVERSING_DUTY_CYCLE = -0.5;

  // IO fields
  private final HopperRollerIO io;
  private final HopperRollerIOInputsAutoLogged inputs = new HopperRollerIOInputsAutoLogged();

  // Enums
  public enum HopperRollerStates {
    OFF,
    ROLLING,
    UNJAMMING,
    PREVENT_JAM,
    REVERSING
  }

  // State variables
  private HopperRollerStates wantedState = HopperRollerStates.OFF;
  private HopperRollerStates currentState = HopperRollerStates.OFF;
  private HopperRollerStates previousState = HopperRollerStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor
  public HopperRoller(HopperRollerIO io) {
    this.io = io;
  }

  // State machine methods

  public HopperRollerStates getState() {
    return currentState;
  }

  public void setWantedState(HopperRollerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case ROLLING:
        currentState = HopperRollerStates.ROLLING;
        break;
      case PREVENT_JAM:
        currentState = HopperRollerStates.PREVENT_JAM;
        break;
      case UNJAMMING:
        currentState = HopperRollerStates.UNJAMMING;
        break;
      case REVERSING:
        currentState = HopperRollerStates.REVERSING;
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
        setVelocity(ROLLER_VELOCITY_RPM);
        break;
      case PREVENT_JAM:
        setDutyCycle(PREVENT_JAM_DUTY_CYCLE);
        break;
      case UNJAMMING:
        setDutyCycle(UNJAMMING_DUTY_CYCLE);
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

  public void setVelocity(double rpm) {
    io.setVelocity(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(double dutyCycle) {
    return runEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/HopperRoller", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Superstructure/Subsystems/HopperRoller/WantedState", wantedState);
    Logger.recordOutput("Superstructure/Subsystems/HopperRoller/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Subsystems/HopperRoller/PreviousState", previousState);
    Logger.recordOutput("Superstructure/Subsystems/HopperRoller/ControlState", controlState);
  }
}
