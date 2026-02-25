package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
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
  public enum ShooterWantedStates {
    IDLE,
    SHOOTING
  }

  public enum ShooterStates {
    PREPARING_TO_FIRE,
    FIRING,
    IDLE
  }

  // Subsystem refs
  private final Flywheel flywheel;
  private final Hood hood;
  private final FlywheelKicker flywheelKicker;
  private final BooleanSupplier isAlignedToTarget;

  // State variables
  private ShooterWantedStates wantedState = ShooterWantedStates.IDLE;
  private ShooterStates currentState = ShooterStates.IDLE;
  private ShooterStates previousState = ShooterStates.IDLE;

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
   * Sets the wanted state of the shooter state machine.
   *
   * @param state the desired shooter state
   */
  public void setWantedState(ShooterWantedStates state) {
    wantedState = state;
  }

  /**
   * Updates the shooter state based on wanted state, subsystem readiness, and alignment. Should be
   * called every cycle by the SuperStructure.
   */
  public void update() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING:
        boolean flywheelReady = flywheel.isReadyToShoot();
        boolean hoodReady = hood.getState() == HoodInternalStates.AT_SETPOINT;
        boolean drivetrainAligned = isAlignedToTarget.getAsBoolean();

        Logger.recordOutput("Superstructure/Shooting/FlywheelReady", flywheelReady);
        Logger.recordOutput("Superstructure/Shooting/HoodReady", hoodReady);
        Logger.recordOutput("Superstructure/Shooting/DrivetrainAligned", drivetrainAligned);

        if (flywheelReady && hoodReady && drivetrainAligned) {
          currentState = ShooterStates.FIRING;
        } else {
          currentState = ShooterStates.PREPARING_TO_FIRE;
        }
        break;
      case IDLE:
      default:
        currentState = ShooterStates.IDLE;
        break;
    }
  }

  /**
   * Applies the current shooter state to subsystems. Sets flywheel, hood, and flywheel kicker
   * wanted states based on the current state.
   */
  public void apply() {
    switch (currentState) {
      case PREPARING_TO_FIRE:
        flywheel.setWantedState(FlywheelWantedStates.SHOOTING);
        hood.setWantedState(HoodWantedStates.AIMING);
        flywheelKicker.setWantedState(FlywheelKickerStates.IDLE);
        break;
      case FIRING:
        flywheel.setWantedState(FlywheelWantedStates.SHOOTING);
        hood.setWantedState(HoodWantedStates.AIMING);
        flywheelKicker.setWantedState(FlywheelKickerStates.KICKING);
        break;
      case IDLE:
      default:
        flywheel.setWantedState(FlywheelWantedStates.IDLE);
        hood.setWantedState(HoodWantedStates.IDLE);
        flywheelKicker.setWantedState(FlywheelKickerStates.IDLE);
        break;
    }
  }

  /** Logs the wanted, current, and previous shooter states. */
  public void log() {
    Logger.recordOutput("Superstructure/WantedShooterState", wantedState.toString());
    Logger.recordOutput("Superstructure/CurrentShooterState", currentState.toString());
    Logger.recordOutput("Superstructure/PreviousShooterState", previousState.toString());
  }
}
