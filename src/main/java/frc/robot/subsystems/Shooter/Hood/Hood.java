// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private static final double TOLERANCE = 0.5;
  private DoubleSupplier hoodAngleSupplier = () -> 0.0;

  public enum HoodWantedStates {
    IDLE,
    AIMING
  }

  public enum HoodStates {
    OFF,
    MOVING,
    AT_SETPOINT
  }

  /**
   * Sets the supplier for the hood angle from the shot calculator.
   *
   * @param hoodAngleSupplier a DoubleSupplier providing the desired hood angle
   */
  public void setHoodAngleSupplier(DoubleSupplier hoodAngleSupplier) {
    this.hoodAngleSupplier = hoodAngleSupplier;
  }

  private HoodWantedStates wantedState = HoodWantedStates.IDLE;
  private HoodStates currentState = HoodStates.OFF;
  private HoodStates previousState = HoodStates.OFF;

  public void setWantedState(HoodWantedStates state) {
    wantedState = state;
  }

  private void applyState() {
    switch (currentState) {
      case MOVING:
      case AT_SETPOINT:
        setPosition(hoodAngleSupplier.getAsDouble());
        break;
      case OFF:
      default:
        setPosition(0.0);
        break;
    }
  }

  public HoodStates getState() {
    return currentState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case AIMING:
        if (atSetpoint(hoodAngleSupplier)) {
          currentState = HoodStates.AT_SETPOINT;
        } else {
          currentState = HoodStates.MOVING;
        }
        break;
      case IDLE:
      default:
        currentState = HoodStates.OFF;
        break;
    }
  }

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return inputs.position;
  }

  public Command setPositionCmd(DoubleSupplier position) {
    return this.run(() -> io.setPosition(position.getAsDouble()));
  }

  public Command setPositionCmd(double position) {
    return this.setPositionCmd(() -> position);
  }

  public void setZero() {
    io.setZero();
  }

  public void stop() {
    io.setDutyCycle(0);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(getPosition() - setpoint) < TOLERANCE;
  }

  public boolean atSetpoint(DoubleSupplier setpoint) {
    return atSetpoint(setpoint.getAsDouble());
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    updateState();
    applyState();
    Logger.recordOutput("Subsystems/Hood/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Hood/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Hood/PreviousState", previousState.toString());
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command zero() {
    return this.runOnce(() -> setZero());
  }
}
