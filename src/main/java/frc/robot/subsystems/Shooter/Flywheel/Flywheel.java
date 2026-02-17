// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public enum FlywheelStates {
    OFF,
    SPINNING_UP,    // Duty cycle bang-bang - fast acceleration
    AT_SPEED,       // Torque current bang-bang - maintaining speed
    SHOOTING        // Legacy state for compatibility
  }


  // These enums are more for logging and debugging purposes - the actual control mode is determined by the state machine logic
  public enum FlywheelControlMode {
    DUTY_CYCLE_BANG_BANG,   // Startup/Recovery - fast acceleration
    TORQUE_CURRENT_BANG_BANG, // Idle/Ball - consistent torque
    COAST // No control, free spinning
  }

  // Tunable parameters for 4-phase bang-bang control
  private static final LoggedTunableNumber toleranceRPS = new LoggedTunableNumber(
      "Flywheel/ToleranceRPS", 0.33);  // ~20 RPM in RPS
  private static final LoggedTunableNumber controlModeDebounceSeconds = new LoggedTunableNumber(
      "Flywheel/ControlModeDebounceSeconds", 0.025);
  private static final LoggedTunableNumber atGoalDebounceSeconds = new LoggedTunableNumber(
      "Flywheel/AtGoalDebounceSeconds", 0.2);

  private final Debouncer controlModeDebouncer = new Debouncer(
      controlModeDebounceSeconds.get(), DebounceType.kFalling);
  private final Debouncer atGoalDebouncer = new Debouncer(
      atGoalDebounceSeconds.get(), DebounceType.kFalling);

  private FlywheelControlMode currentControlMode = FlywheelControlMode.DUTY_CYCLE_BANG_BANG;
  private boolean atGoal = false;
  private double targetVelocityRPS = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  /**
   * Set flywheel velocity using 4-phase bang-bang control.
   * State machine handles control mode transitions automatically.
   */
  public void setVelocityRPS(double velocityRPS) {
    targetVelocityRPS = velocityRPS;
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public FlywheelControlMode getControlMode() {
    return currentControlMode;
  }

  public FlywheelStates getState() {
    return currentState;
  }

  private FlywheelStates wantedState = FlywheelStates.OFF;
  private FlywheelStates currentState = FlywheelStates.OFF;
  private FlywheelStates previousState = FlywheelStates.OFF;

  private void updateState() {
    previousState = currentState;

    // Get current velocity in RPS (native units)
    double currentRPS = getLeaderVelocityRPS();
    
    // Check if velocity is within tolerance
    boolean inTolerance = Math.abs(currentRPS - targetVelocityRPS) <= toleranceRPS.get();
    
    // Update debounced values
    boolean isAtSpeed = controlModeDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    // State machine transitions
    switch (wantedState) {
      case SHOOTING:
        // SHOOTING uses duty cycle bang-bang for maximum acceleration during shot
        currentState = FlywheelStates.SHOOTING;
        break;
      
      case SPINNING_UP:
      case AT_SPEED:
        // Automatic transitions based on velocity
        if (targetVelocityRPS == 0.0) {
          currentState = FlywheelStates.OFF;
        } else if (isAtSpeed) {
          currentState = FlywheelStates.AT_SPEED;
        } else {
          currentState = FlywheelStates.SPINNING_UP;
        }
        break;
      
      case OFF:
        currentState = FlywheelStates.OFF;
        break;
    }
  }

  public double getLeaderVelocityRPS() {
    if (inputs.velocities.length > 0) {
      return inputs.velocities[0];
    }
    return 0.0;
  }

  public boolean atSetpoint(double targetRPS, double toleranceRPS) {
    return Math.abs(getLeaderVelocityRPS() - targetRPS) < toleranceRPS;
  }

  private void applyState() {
    switch (currentState) {
      case SHOOTING:
        // SHOOTING: Duty cycle bang-bang (fast acceleration during shot)
        currentControlMode = FlywheelControlMode.TORQUE_CURRENT_BANG_BANG;
        io.setVelocityBangBang(targetVelocityRPS);
        break;
      
      case SPINNING_UP:
        // SPINNING_UP: Torque current bang-bang (consistent torque while accelerating)
        currentControlMode = FlywheelControlMode.DUTY_CYCLE_BANG_BANG;
        io.setVelocityTorqueCurrentBangBang(targetVelocityRPS);
        break;
      
      case AT_SPEED:
        // AT_SPEED: Torque current bang-bang (maintain speed with consistent torque)
        currentControlMode = FlywheelControlMode.TORQUE_CURRENT_BANG_BANG;
        io.setVelocityTorqueCurrentBangBang(targetVelocityRPS);
        break;

      case OFF:
      default:
        currentControlMode = FlywheelControlMode.COAST;
        io.stop();
        break;
    }
  }

  public void setWantedState(FlywheelStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState.toString());
    Logger.recordOutput("Subsystems/Flywheel/ControlMode", currentControlMode.toString());
    Logger.recordOutput("Subsystems/Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Subsystems/Flywheel/TargetVelocityRPS", targetVelocityRPS);
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.stop();
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.stop());
  }

  public Command setVelocityCommand(double rps) {
    return this.runEnd(() -> setVelocityRPS(rps), () -> io.stop());
  }
}
