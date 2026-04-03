// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRoller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  // Motor output constants
  private static final double INTAKE_VELOCITY_RPM = 4000.0;
  private static final double WOODBOT_INTAKING_DUTY_CYCLE = 0.7;
  private static final double INTAKING_DUTY_CYCLE = 0.8;
  private static final double SHOOT_ASSIST_DUTY_CYCLE = 0.3;
  private static final double REVERSE_VELOCITY_RPM = -3250.0;
  private static final double REVERSE_UNJAM_DUTY_CYCLE = -0.5;

  // Jam detection constants
  private static final double JAM_STATOR_CURRENT_THRESHOLD_AMPS = 50.0;
  private static final double JAM_VELOCITY_THRESHOLD_RPM = 50.0;
  private static final double JAM_DURATION_SECONDS = 0.25;
  private static final double UNJAM_DURATION_SECONDS = 0.15;
  private static final int MAX_UNJAM_ATTEMPTS = 3;

  // IO fields
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  // Jam tracking fields
  private final Timer stallTimer = new Timer();
  private final Timer unjamTimer = new Timer();
  private int unjamAttempts = 0;

  // Enums
  public enum IntakeRollerStates {
    IDLE,
    INTAKING,
    ASSIST_SHOOTING,
    REVERSING,
    JAMMED,
    UNJAMMING
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

  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case INTAKING:
        if (currentState == IntakeRollerStates.UNJAMMING) {
          handleUnjamming();
        } else if (currentState == IntakeRollerStates.JAMMED) {
          handleJammed();
        } else if (isJammed()) {
          stallTimer.stop();
          stallTimer.reset();
          currentState = IntakeRollerStates.JAMMED;
        } else {
          currentState = IntakeRollerStates.INTAKING;
        }
        break;
      case ASSIST_SHOOTING:
        resetJamState();
        unjamAttempts = 0;
        currentState = IntakeRollerStates.ASSIST_SHOOTING;
        break;
      case REVERSING:
        resetJamState();
        unjamAttempts = 0;
        currentState = IntakeRollerStates.REVERSING;
        break;
      case IDLE:
      default:
        resetJamState();
        unjamAttempts = 0;
        currentState = IntakeRollerStates.IDLE;
        break;
    }
  }

  /**
   * Called each cycle while currentState is UNJAMMING. Waits for the unjam window to expire, then
   * returns to INTAKING.
   */
  private void handleUnjamming() {
    if (unjamTimer.get() >= UNJAM_DURATION_SECONDS) {
      resetJamState();
      currentState = IntakeRollerStates.INTAKING;
    }
  }

  /**
   * Called each cycle while currentState is JAMMED. Attempts up to MAX_UNJAM_ATTEMPTS unjam
   * cycles; gives up and stays stopped if the limit is reached.
   */
  private void handleJammed() {
    if (unjamAttempts >= MAX_UNJAM_ATTEMPTS) {
      // Give up — too many failed unjam cycles, stay stopped
      currentState = IntakeRollerStates.JAMMED;
    } else {
      unjamAttempts++;
      unjamTimer.stop();
      unjamTimer.reset();
      unjamTimer.start();
      currentState = IntakeRollerStates.UNJAMMING;
    }
  }

  private void resetJamState() {
    stallTimer.stop();
    stallTimer.reset();
    unjamTimer.stop();
    unjamTimer.reset();
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
      case UNJAMMING:
        unjamming();
        break;
      case JAMMED:
        stop();
        break;
      case IDLE:
      default:
        stop();
        break;
    }
  }

  private void shootAssist() {
    setDutyCycle(SHOOT_ASSIST_DUTY_CYCLE);
  }

  private void intaking() {
    if (Constants.getRobotType() != Constants.RobotType.WOODBOT) {
      setVelocity(INTAKE_VELOCITY_RPM);
    } else {
      setDutyCycle(WOODBOT_INTAKING_DUTY_CYCLE);
    }
  }

  private void reversing() {
    setVelocity(REVERSE_VELOCITY_RPM);
  }

  private void unjamming() {
    setDutyCycle(REVERSE_UNJAM_DUTY_CYCLE);
  }

  private boolean isJammed() {
    boolean highCurrent =
        inputs.statorCurrent[0] > JAM_STATOR_CURRENT_THRESHOLD_AMPS
            && inputs.statorCurrent[1] > JAM_STATOR_CURRENT_THRESHOLD_AMPS;
    boolean lowVelocity =
        Math.abs(inputs.velocity[0]) < JAM_VELOCITY_THRESHOLD_RPM
            && Math.abs(inputs.velocity[1]) < JAM_VELOCITY_THRESHOLD_RPM;

    if (highCurrent && lowVelocity) {
      stallTimer.start();
    } else {
      stallTimer.stop();
      stallTimer.reset();
    }

    return stallTimer.get() >= JAM_DURATION_SECONDS;
  }

  // IO delegation methods

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  /**
   * Updates the closed-loop PID and feedforward gains.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kV Velocity feedforward gain
   * @param kS Static feedforward gain
   */
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    io.setPID(kP, kI, kD, kV, kS);
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
    Logger.recordOutput("Subsystems/IntakeRoller/UnjamAttempts", unjamAttempts);
  }
}
