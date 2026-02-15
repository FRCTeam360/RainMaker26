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
  private static final double SPINUP_SHOOTING_HOOD_POSITION_DEGREES = 8.0;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final double TOLERANCE = 0.5;
  private DoubleSupplier hoodAngleSupplier = () -> 0.0;

  public enum HoodStates {
    OFF,
    SHOOTING,
    AIMING
  }

  /**
   * Sets the supplier for the hood angle from the shot calculator.
   *
   * @param hoodAngleSupplier a DoubleSupplier providing the desired hood angle
   */
  public void setHoodAngleSupplier(DoubleSupplier hoodAngleSupplier) {
    this.hoodAngleSupplier = hoodAngleSupplier;
  }

  private HoodStates wantedState = HoodStates.OFF;
  private HoodStates currentState = HoodStates.OFF;
  private HoodStates previousState = HoodStates.OFF;

  public void setWantedState(HoodStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  private void applyState() {
    switch (currentState) {
      case SHOOTING:
        setPosition(SPINUP_SHOOTING_HOOD_POSITION_DEGREES);
        break;
      case AIMING:
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
      case SHOOTING:
        currentState = HoodStates.SHOOTING;
        break;
      case AIMING:
        currentState = HoodStates.AIMING;
        break;
      case OFF:
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

  public void setEncoder(double position) {
    io.setEncoder(position);
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
    final double zeroDutyCycle = -0.03;
    final double zeroTimeoutSeconds = 3.0;
    final double zeroSettleSeconds = 2.0;
    return Commands.runEnd(() -> io.setDutyCycle(zeroDutyCycle), () -> io.setDutyCycle(0.0))
        .withTimeout(zeroTimeoutSeconds)
        .andThen(Commands.waitSeconds(zeroSettleSeconds))
        .andThen(zero());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
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
    return this.runOnce(() -> setEncoder(0.0));
  }
}
