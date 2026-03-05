package frc.robot.subsystems.Intake;

import frc.robot.subsystems.Intake.IntakePivot.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.IntakePivot.IntakePivotWantedStates;
import frc.robot.subsystems.Intake.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.Intake.IntakeRoller.IntakeRoller.IntakeRollerStates;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the intake state machine, coordinating the intake rollers and intake pivot to centralize
 * intake logic. Called by the SuperStructure each cycle via update(), apply(), and log().
 *
 * <p>The intake state is controlled exclusively by driver inputs through the SuperStructure's
 * setIntakeState() / setIntakeStateCommand() API. The SuperStructure's internal state helpers
 * (shooting, idle, etc.) do not write to this state machine.
 */
public class IntakeStateMachine {
  // Enums

  /** Wanted states set by driver inputs to request intake behavior. */
  public enum IntakeWantedStates {
    IDLE,
    INTAKING,
    STOWED,
    AGITATING,
    DEPLOYED
  }

  /** Internal states representing the resolved intake behavior. */
  public enum IntakeInternalStates {
    IDLE,
    INTAKING,
    STOWED,
    AGITATING,
    DEPLOYED
  }

  // Subsystem refs
  private final IntakeRoller intakeRoller;
  private final IntakePivot intakePivot;

  // State variables
  private IntakeWantedStates wantedState = IntakeWantedStates.IDLE;
  private IntakeInternalStates currentState = IntakeInternalStates.IDLE;
  private IntakeInternalStates previousState = IntakeInternalStates.IDLE;

  /**
   * Creates a new IntakeStateMachine.
   *
   * @param intakeRoller the intake roller subsystem
   * @param intakePivot the intake pivot subsystem
   */
  public IntakeStateMachine(IntakeRoller intakeRoller, IntakePivot intakePivot) {
    this.intakeRoller = intakeRoller;
    this.intakePivot = intakePivot;
  }

  /**
   * Sets the wanted state of the intake state machine.
   *
   * @param state the desired intake state
   */
  public void setWantedState(IntakeWantedStates state) {
    wantedState = state;
  }

  /** Returns the current intake internal state. */
  public IntakeInternalStates getState() {
    return currentState;
  }

  /**
   * Updates the intake state based on wanted state. Should be called every cycle by the
   * SuperStructure.
   */
  public void update() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = IntakeInternalStates.INTAKING;
        break;
      case STOWED:
        currentState = IntakeInternalStates.STOWED;
        break;
      case AGITATING:
        currentState = IntakeInternalStates.AGITATING;
        break;
      case DEPLOYED:
        currentState = IntakeInternalStates.DEPLOYED;
      case IDLE:
      default:
        currentState = IntakeInternalStates.IDLE;
        break;
    }
  }

  /**
   * Applies the current intake state to the intake and intake pivot subsystems. Sets their wanted
   * states based on the resolved internal state.
   */
  public void apply() {
    switch (currentState) {
      case DEPLOYED:
        intakePivot.setWantedState(IntakePivotWantedStates.DEPLOYED);
        break;
      case INTAKING:
        intakeRoller.setWantedState(IntakeRollerStates.INTAKING);
        intakePivot.setWantedState(IntakePivotWantedStates.DEPLOYED);
        break;
      case AGITATING:
        intakeRoller.setWantedState(IntakeRollerStates.ASSIST_SHOOTING);
        intakePivot.setWantedState(IntakePivotWantedStates.AGITATE_HOPPER);
        break;
      case STOWED:
        intakeRoller.setWantedState(IntakeRollerStates.IDLE);
        intakePivot.setWantedState(IntakePivotWantedStates.STOWED);
        break;
      case IDLE:
      default:
        intakeRoller.setWantedState(IntakeRollerStates.IDLE);
        intakePivot.setWantedState(IntakePivotWantedStates.IDLE);
        break;
    }
  }

  /** Logs the wanted, current, and previous intake states. */
  public void log() {
    Logger.recordOutput("Superstructure/WantedIntakeState", wantedState);
    Logger.recordOutput("Superstructure/CurrentIntakeState", currentState);
    Logger.recordOutput("Superstructure/PreviousIntakeState", previousState);
  }
}
