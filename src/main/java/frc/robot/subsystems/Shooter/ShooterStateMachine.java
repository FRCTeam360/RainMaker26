package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelInternalStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelWantedStates;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodInternalStates;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodWantedStates;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the shooter state machine, coordinating the flywheel, hood, and flywheel kicker to
 * determine when the shooter is ready to fire.
 */
public class ShooterStateMachine {
  // Enums
  public enum ShooterStates {
    PREPARING,
    FIRING,
    IDLE
  }

  // Subsystem refs
  private final Flywheel flywheel;
  private final Hood hood;
  private final FlywheelKicker flywheelKicker;
  private final BooleanSupplier isAlignedToTarget;

  // State variables
  private ShooterStates currentState = ShooterStates.PREPARING;
  private ShooterStates previousState = ShooterStates.PREPARING;

  /**
   * Creates a new ShooterStateMachine.
   *
   * @param flywheel the flywheel subsystem
   * @param hood the hood subsystem
   * @param flywheelKicker the flywheel kicker subsystem
   * @param isAlignedToTarget supplier that returns true when the robot is aligned to the target
   */
  public ShooterStateMachine(
      Flywheel flywheel,
      Hood hood,
      FlywheelKicker flywheelKicker,
      BooleanSupplier isAlignedToTarget) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.flywheelKicker = flywheelKicker;
    this.isAlignedToTarget = isAlignedToTarget;
  }

  /** Returns the current shooter state. */
  public ShooterStates getState() {
    return currentState;
  }

  /**
   * Updates the shooter state based on subsystem readiness and alignment. Should be called by the
   * SuperStructure when in a shooting super state.
   */
  public void update() {
    previousState = currentState;

    boolean flywheelReady = flywheel.getState() == FlywheelInternalStates.AT_SETPOINT;
    boolean hoodReady = hood.getState() == HoodInternalStates.AT_SETPOINT;
    boolean aligned = isAlignedToTarget.getAsBoolean();

    Logger.recordOutput("Superstructure/Shooting/FlywheelReady", flywheelReady);
    Logger.recordOutput("Superstructure/Shooting/HoodReady", hoodReady);
    Logger.recordOutput("Superstructure/Shooting/Aligned", aligned);

    if (flywheelReady && hoodReady && aligned) {
      currentState = ShooterStates.FIRING;
    } else {
      currentState = ShooterStates.PREPARING;
    }
  }

  /**
   * Applies the current shooter state to subsystems. Sets flywheel and hood to SHOOTING, and
   * controls the flywheel kicker based on whether we are FIRING or PREPARING.
   */
  public void apply() {
    flywheel.setWantedState(FlywheelWantedStates.SHOOTING);
    hood.setWantedState(HoodWantedStates.SHOOTING);

    if (currentState == ShooterStates.FIRING) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SHOOTING);
    } else {
      flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
    }
  }

  /** Resets the shooter state to PREPARING. Called when the robot exits a shooting super state. */
  public void reset() {
    currentState = ShooterStates.IDLE;
  }

  /** Stops all controlled subsystems */
  public void stop() {
    flywheelKicker.setWantedState(FlywheelKicker.FlywheelKickerStates.OFF);
    flywheel.setWantedState(FlywheelWantedStates.IDLE);
    hood.setWantedState(HoodWantedStates.IDLE);
  }

  /** Logs the current and previous shooter states. */
  public void log() {
    Logger.recordOutput("Superstructure/PreviousShooterState", previousState.toString());
    Logger.recordOutput("Superstructure/CurrentShooterState", currentState.toString());
  }
}
