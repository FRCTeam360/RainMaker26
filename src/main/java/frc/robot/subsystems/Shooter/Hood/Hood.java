// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  // Constants
  private static final double TOLERANCE = 0.5;

  // IO fields
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  // Other fields
  private DoubleSupplier hoodAngleSupplier = () -> 0.0;
  private BooleanSupplier shouldDuck = () -> false;

  // Enums
  public enum HoodWantedStates {
    IDLE,
    AIMING,
    PASSIVE_PREP
  }

  public enum HoodInternalStates {
    OFF,
    MOVING,
    AT_SETPOINT,
    DUCKING
  }

  // State variables
  private HoodWantedStates wantedState = HoodWantedStates.IDLE;
  private HoodInternalStates currentState = HoodInternalStates.OFF;
  private HoodInternalStates previousState = HoodInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  // State machine methods

  public HoodInternalStates getState() {
    return currentState;
  }

  public void setWantedState(HoodWantedStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  /**
   * Sets the supplier for the hood angle from the shot calculator.
   *
   * @param hoodAngleSupplier a DoubleSupplier providing the desired hood angle
   */
  public void setHoodAngleSupplier(DoubleSupplier hoodAngleSupplier) {
    this.hoodAngleSupplier = hoodAngleSupplier;
  }

  /**
   * Sets the supplier that determines whether the hood should duck to zero in PASSIVE_PREP state.
   *
   * @param shouldDuck a BooleanSupplier returning true when the hood should retract to zero
   */
  public void setShouldDuckSupplier(BooleanSupplier shouldDuck) {
    this.shouldDuck = shouldDuck;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case AIMING:
        holdShootingPosition();
        break;
      case PASSIVE_PREP:
        if (shouldDuck.getAsBoolean()) {
          currentState = HoodInternalStates.DUCKING; // quack!
        } else {
          holdShootingPosition();
        }
        break;
      case IDLE:
      default:
        currentState = HoodInternalStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case MOVING:
      case AT_SETPOINT:
        setPosition(hoodAngleSupplier.getAsDouble());
        break;
      case DUCKING:
        moveHoodToZero();
        break;
      case OFF:
      default:
        setDutyCycle(0.0);
        break;
    }
  }

  // Subsystem state helpers
  private void moveHoodToZero() {
    setPosition(0.0);
  }

  private void holdShootingPosition() {
    if (atSetpoint(hoodAngleSupplier)) {
      currentState = HoodInternalStates.AT_SETPOINT;
    } else {
      currentState = HoodInternalStates.MOVING;
    }
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(getPosition() - setpoint) < TOLERANCE;
  }

  public boolean atSetpoint(DoubleSupplier setpoint) {
    return atSetpoint(setpoint.getAsDouble());
  }

  // IO delegation methods

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return inputs.position;
  }

  public void setZero() {
    io.setZero();
  }

  public void stop() {
    io.setDutyCycle(0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return this.run(() -> io.setPosition(position.getAsDouble()));
  }

  public Command setPositionCommand(double position) {
    return this.setPositionCommand(() -> position);
  }

  public Command zero() {
    return this.runOnce(() -> setZero());
  }

  public Command moveToZeroAndZero() {
    final double ZERO_DUTY_CYCLE = -0.03;
    final double ZERO_TIMEOUT_SECONDS = 3.0;
    final double ZERO_SETTLE_SECONDS = 2.0;
    return Commands.runEnd(() -> io.setDutyCycle(ZERO_DUTY_CYCLE), () -> io.setDutyCycle(0.0))
        .withTimeout(ZERO_TIMEOUT_SECONDS)
        .andThen(Commands.waitSeconds(ZERO_SETTLE_SECONDS))
        .andThen(zero());
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/Hood/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Hood/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Hood/PreviousState", previousState);
    Logger.recordOutput("Subsystems/Hood/ControlState", controlState);
  }
}
