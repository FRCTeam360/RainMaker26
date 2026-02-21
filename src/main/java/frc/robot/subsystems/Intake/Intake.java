// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static final double INTAKE_VELOCITY_RPM = 4500.0;
  private static final double JAMMED_SUPPLY_CURRENT_DRAW = 35.0;
  private static final double REVERSE_UNJAM_DUTY_CYCLE = -0.5;
  private static final double INTAKE_DUTY_CYCLE = 0.75;

  public enum IntakeStates {
    OFF,
    INTAKING,
    SHOOTING,
    // JAMMED
  }

  private IntakeStates wantedState = IntakeStates.OFF;
  private IntakeStates currentState = IntakeStates.OFF;
  private IntakeStates previousState = IntakeStates.OFF;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public IntakeStates getState() {
    return currentState;
  }

  public void setWantedState(IntakeStates state) {
    wantedState = state;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        // if (isJammed()) {
        //   currentState = IntakeStates.JAMMED;
        // } else {
        // }
        currentState = IntakeStates.INTAKING;
        break;

      case SHOOTING:
        currentState = IntakeStates.SHOOTING;
        break;
      case OFF:
      default:
        currentState = IntakeStates.OFF;
        break;
        // case JAMMED:
        //   currentState = IntakeStates.JAMMED;
    }
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        intaking();
        break;
      case SHOOTING:
        intaking();
        break;
      case OFF:
      default:
        stop();
        break;
        // case JAMMED:
        //   unjamIntake();
    }
  }

  private void intaking() {
    setDutyCycle(INTAKE_DUTY_CYCLE);
  }

  // private void unjamIntake() {
  //   if (isJammed()) {
  //     this.setDutyCycle(REVERSE_UNJAM_DUTY_CYCLE);
  //   }
  // }

  // private boolean isJammed() {
  //   return inputs.supplyCurrent >= JAMMED_SUPPLY_CURRENT_DRAW;
  // }

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public Command setVelocityCommand(double velocity) {
    return this.runEnd(() -> setVelocity(velocity), () -> setVelocity(0.0));
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    updateState();
    applyState();
    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Intake/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Intake/PreviousState", previousState.toString());
    // Logger.recordOutput("Subsystems/Intake/PreviousState", isJammed());
  }
}
