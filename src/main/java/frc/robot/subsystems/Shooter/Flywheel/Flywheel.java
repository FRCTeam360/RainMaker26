// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // Constants
  private static final double TOLERANCE_RPM = 100.0;
  private static final double BALL_FIRED_DEBOUNCE_SECONDS = 0.04;
  private static final double SUSTAINED_RPM_DROP_DEBOUNCE_SECOND = 0.2;

  private long launchCount = 0;

  // IO fields
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Other fields
  private DoubleSupplier shootVelocitySupplier = () -> 0.0;

  // Enums
  public enum FlywheelWantedStates {
    IDLE,
    SHOOTING
  }

  public enum FlywheelInternalStates {
    OFF,
    SPINNING_UP,
    AT_SETPOINT,
    UNDER_SHOOTING
  }

  // ballFiredDebouncer: prevents rapid spinup/hold switching when velocity briefly dips below
  // tolerance due to noise. Also serves as the shot detection signal — a sustained drop past
  // this debounce window indicates a ball has passed through the flywheel.
  private final Debouncer ballFiredDebouncer =
      new Debouncer(BALL_FIRED_DEBOUNCE_SECONDS, DebounceType.kFalling);
  // underspeedDebouncer: detects when RPM has been below tolerance for too long, indicating too
  // many balls have passed through in rapid succession and the flywheel can't recover between
  // shots. kFalling means the drop detection is delayed, so single-shot dips don't trigger it.
  private final Debouncer underspeedDebouncer =
      new Debouncer(SUSTAINED_RPM_DROP_DEBOUNCE_SECOND, DebounceType.kFalling);

  // State variables
  private FlywheelWantedStates wantedState = FlywheelWantedStates.IDLE;
  private FlywheelInternalStates currentState = FlywheelInternalStates.OFF;
  private FlywheelInternalStates previousState = FlywheelInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  // State machine methods

  public FlywheelInternalStates getState() {
    return currentState;
  }

  public void setWantedState(FlywheelWantedStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  /**
   * Sets the supplier for the shoot velocity from the shot calculator.
   *
   * @param shootVelocitySupplier a DoubleSupplier providing the desired flywheel velocity in RPM
   */
  public void setShootVelocitySupplier(DoubleSupplier shootVelocitySupplier) {
    this.shootVelocitySupplier = shootVelocitySupplier;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING:
        double targetRPM = shootVelocitySupplier.getAsDouble();
        boolean atBangBangSetpoint = atSetpoint(targetRPM);
        boolean underspeed = isUnderspeed(targetRPM);
        boolean wasAtSetpoint = previousState == FlywheelInternalStates.AT_SETPOINT;
        boolean ballFired = wasAtSetpoint && !atBangBangSetpoint;

        if (underspeed) {
          currentState = FlywheelInternalStates.UNDER_SHOOTING;
        } else if (ballFired) {
          launchCount++;
          currentState = FlywheelInternalStates.SPINNING_UP;
        } else if (atBangBangSetpoint) {
          currentState = FlywheelInternalStates.AT_SETPOINT;
        } else {
          currentState = FlywheelInternalStates.SPINNING_UP;
        }
        break;
      case IDLE:
      default:
        currentState = FlywheelInternalStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case SPINNING_UP:
      case UNDER_SHOOTING:
        setSpinupVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case AT_SETPOINT:
        setHoldVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case OFF:
      default:
        setDutyCycle(0.0);
        break;
    }
  }

  private boolean atSetpoint(double targetRPM) {
    if (inputs.velocities.length > 0) {
      boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
      return ballFiredDebouncer.calculate(inTolerance);
    }
    return false;
  }

  /**
   * Returns whether the flywheel RPM has been below tolerance for too long, indicating too many
   * balls have passed through and the flywheel can't recover between shots.
   *
   * @param targetRPM the target velocity in RPM
   * @return true if the flywheel is underspeed for a sustained period
   */
  private boolean isUnderspeed(double targetRPM) {
    if (inputs.velocities.length > 0) {
      boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
      return !underspeedDebouncer.calculate(inTolerance);
    }
    return true;
  }

  // IO delegation methods

  private void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void setSpinupVelocityControl(double rpm) {
    io.setSpinupVelocityControl(rpm);
  }

  public void setHoldVelocityControl(double rpm) {
    io.setHoldVelocityControl(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> setDutyCycle(valueSup.getAsDouble()), () -> setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double rpm) {
    return this.runEnd(() -> setHoldVelocityControl(rpm), () -> setDutyCycle(0.0));
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(
        () -> setHoldVelocityControl(supplierVelocity.getAsDouble()), () -> setDutyCycle(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      // Update state machine on every cycle to respond to velocity/current state changes
      updateState();
      applyState();
    }

    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState);
    Logger.recordOutput("Subsystems/Flywheel/ControlState", controlState);
    Logger.recordOutput("Subsystems/Flywheel/LaunchCount", launchCount);
  }
}
