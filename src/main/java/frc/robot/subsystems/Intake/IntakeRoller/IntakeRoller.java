// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  // Constants
  private static final double INTAKE_VELOCITY_RPM = 4500.0;
  private static final double JAMMED_SUPPLY_CURRENT_DRAW = 35.0;
  private static final double REVERSE_UNJAM_DUTY_CYCLE = -0.5;
  private static final double INTAKING_DUTY_CYCLE = 0.8;
  private static final double SHOOT_ASSIST_DUTY_CYCLE = 0.3;
  private static final double REVERSE_DUTY_CYCLE = -0.3;

  // IO fields
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  // Other fields
  private DoubleSupplier dutyCycleSupplier = () -> INTAKING_DUTY_CYCLE;

  // Enums
  public enum IntakeRollerStates {
    IDLE,
    INTAKING,
    ASSIST_SHOOTING,
    REVERSING
    // JAMMED
  }

  // State variables
  private IntakeRollerStates wantedState = IntakeRollerStates.IDLE;
  private IntakeRollerStates currentState = IntakeRollerStates.IDLE;
  private IntakeRollerStates previousState = IntakeRollerStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new IntakeRoller. */
  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  // State machine methods

  public IntakeRollerStates getState() {
    return currentState;
  }

  public void setWantedState(IntakeRollerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  public void setDutyCycleSupplier(DoubleSupplier dutyCycleSupplier) {
    this.dutyCycleSupplier = dutyCycleSupplier;
  }

  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        // if (isJammed()) {
        //   currentState = IntakeRollerStates.JAMMED;
        // } else {
        // }
        currentState = IntakeRollerStates.INTAKING;
        break;
      case ASSIST_SHOOTING:
        currentState = IntakeRollerStates.ASSIST_SHOOTING;
        break;
      case REVERSING:
        currentState = IntakeRollerStates.REVERSING;
        break;
      case IDLE:
      default:
        currentState = IntakeRollerStates.IDLE;
        break;
        // case JAMMED:
        //   currentState = IntakeRollerStates.JAMMED;
    }
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        intaking();
        break;
      case ASSIST_SHOOTING:
        shootAssist();
        break;
      case REVERSING:
        reversing();
        break;
      case IDLE:
      default:
        stop();
        break;
        // case JAMMED:
        //   unjamIntake();
    }
  }

  private void shootAssist() {
    setDutyCycle(SHOOT_ASSIST_DUTY_CYCLE);
  }

  private void intaking() {
    if (Constants.getRobotType() != Constants.RobotType.WOODBOT) {
      setVelocity(2000);
    } else {
      setDutyCycle(0.7);
    }
  }

  private void reversing() {
    setDutyCycle(REVERSE_DUTY_CYCLE);
  }

  // private void unjamIntake() {
  //   if (isJammed()) {
  //     this.setDutyCycle(REVERSE_UNJAM_DUTY_CYCLE);
  //   }
  // }

  // private boolean isJammed() {
  //   return inputs.supplyCurrent >= JAMMED_SUPPLY_CURRENT_DRAW;
  // }

  // IO delegation methods

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double velocity) {
    return this.runEnd(() -> setVelocity(velocity), () -> setVelocity(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/IntakeRoller/WantedState", wantedState);
    Logger.recordOutput("Subsystems/IntakeRoller/CurrentState", currentState);
    Logger.recordOutput("Subsystems/IntakeRoller/PreviousState", previousState);
    Logger.recordOutput("Subsystems/IntakeRoller/ControlState", controlState);
    // Logger.recordOutput("Subsystems/IntakeRoller/PreviousState", isJammed());
  }
}
