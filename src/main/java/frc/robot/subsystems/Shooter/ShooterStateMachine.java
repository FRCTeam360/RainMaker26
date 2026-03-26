package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
  private boolean isForced = false;

  // Enums
  public enum ShooterWantedStates {
    IDLE,
    SHOOTING,
    PASSIVE_SHOOTER,
    REVERSING,
    FORCED_SHOT
  }

  public enum ShooterStates {
    PREPARING_TO_FIRE,
    AIMED,
    FIRING,
    WAITING,
    STANDBY,
    UNJAMMING,
    IDLE
  }

  // Subsystem refs
  private final Flywheel flywheel;
  private final Hood hood;
  private final FlywheelKicker flywheelKicker;
  private final BooleanSupplier isAlignedToTarget;
  private final BooleanSupplier canShootToTarget;
  private BooleanSupplier isInAllianceZone = () -> false;

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
      BooleanSupplier isAlignedToTarget,
      BooleanSupplier canShootToTarget) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.flywheelKicker = flywheelKicker;
    this.isAlignedToTarget = isAlignedToTarget;
    this.canShootToTarget = canShootToTarget;
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

  public void setIsInAllianceZoneSupplier(BooleanSupplier supplier) {
    this.isInAllianceZone = supplier;
  }

  /**
   * Updates the shooter state based on wanted state, subsystem readiness, and alignment. Should be
   * called every cycle by the SuperStructure.
   *
   * <p>Uses the flywheel's internal state transitions to gate firing:
   *
   * <ul>
   *   <li>{@link FlywheelInternalStates#AT_SETPOINT} — flywheel velocity is sustained in tolerance;
   *       combined with hood and drivetrain readiness, transitions to FIRING
   *   <li>{@link FlywheelInternalStates#UNDER_SHOOTING} — sustained RPM drop detected from a shot
   *       passing through; reverts to PREPARING_TO_FIRE to restart the cycle
   * </ul>
   */
  public void update() {
    previousState = currentState;

    switch (wantedState) {
      case FORCED_SHOT:
        isForced = true;
      case SHOOTING:
        FlywheelInternalStates flywheelState = flywheel.getState();
        boolean flywheelReady = flywheelState == FlywheelInternalStates.AT_SETPOINT;
        boolean flywheelUnderShooting = flywheelState == FlywheelInternalStates.UNDER_SHOOTING;
        boolean hoodReady = hood.getState() == HoodInternalStates.AT_SETPOINT;
        boolean drivetrainAligned = isForced ? true : isAlignedToTarget.getAsBoolean();
        boolean targetReady = isForced ? true : canShootToTarget.getAsBoolean();
        isForced = false;

        Logger.recordOutput("Superstructure/Shooting/FlywheelState", flywheelState);
        // SmartDashboard.putString("Superstructure/Shooting/FlywheelState",
        // flywheelState.toString());
        Logger.recordOutput("Superstructure/Shooting/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("Superstructure/Shooting/FlywheelReady", flywheelReady);
        Logger.recordOutput("Superstructure/Shooting/HoodReady", hoodReady);
        SmartDashboard.putBoolean("Superstructure/Shooting/HoodReady", hoodReady);
        Logger.recordOutput("Superstructure/Shooting/DrivetrainAligned", drivetrainAligned);
        Logger.recordOutput("Superstructure/Shooting/TargetReady", targetReady);
        SmartDashboard.putBoolean("Superstructure/Shooting/DrivetrainAligned", drivetrainAligned);

        // AIMED requires flywheel and hood to be genuinely at setpoint.
        // Bang-bang oscillations during firing are tolerated to stay in FIRING —
        // only revert when UNDER_SHOOTING signals a sustained RPM drop.
        boolean inBangBang = previousState == ShooterStates.FIRING && !flywheelUnderShooting;
        boolean subsystemsReady = flywheelReady && hoodReady && drivetrainAligned;
        boolean shouldFire =
            (flywheelReady || inBangBang) && hoodReady && drivetrainAligned && targetReady;

        Logger.recordOutput("Superstructure/Shooting/InBangBang", inBangBang);
        Logger.recordOutput("Superstructure/Shooting/SubsystemsReady", subsystemsReady);
        Logger.recordOutput("Superstructure/Shooting/ShouldFire", shouldFire);

        if (shouldFire) {
          currentState = ShooterStates.FIRING;
        } else if (subsystemsReady) {
          currentState = ShooterStates.AIMED;
        } else {
          currentState = ShooterStates.PREPARING_TO_FIRE;
        }
        break;
      case PASSIVE_SHOOTER:
        currentState =
            isInAllianceZone.getAsBoolean() ? ShooterStates.STANDBY : ShooterStates.WAITING;
        break;
      case REVERSING:
        currentState = ShooterStates.UNJAMMING;
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
      case AIMED:
        flywheel.setWantedState(FlywheelWantedStates.SHOOTING);
        hood.setWantedState(HoodWantedStates.AIMING);
        if (Constants.getRobotType() != Constants.RobotType.WOODBOT) {
          flywheelKicker.setWantedState(FlywheelKickerStates.KICKING);
        } else {
          flywheelKicker.setWantedState(FlywheelKickerStates.IDLE);
        }
        break;
      case FIRING:
        flywheel.setWantedState(FlywheelWantedStates.SHOOTING);
        hood.setWantedState(HoodWantedStates.AIMING);
        flywheelKicker.setWantedState(FlywheelKickerStates.KICKING);
        break;
      case WAITING:
        flywheel.setWantedState(FlywheelWantedStates.IDLE);
        hood.setWantedState(HoodWantedStates.DUCKED);
        flywheelKicker.setWantedState(FlywheelKickerStates.IDLE);
        break;
      case STANDBY:
        flywheel.setWantedState(FlywheelWantedStates.COASTING);
        hood.setWantedState(HoodWantedStates.DUCKED);
        flywheelKicker.setWantedState(FlywheelKickerStates.IDLE);
        break;
      case UNJAMMING:
        flywheel.setWantedState(FlywheelWantedStates.IDLE);
        hood.setWantedState(HoodWantedStates.IDLE);
        flywheelKicker.setWantedState(FlywheelKickerStates.REVERSING);
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
    Logger.recordOutput("Superstructure/WantedShooterState", wantedState);
    SmartDashboard.putString("Superstructure/WantedShooterState", wantedState.toString());
    Logger.recordOutput("Superstructure/CurrentShooterState", currentState);
    SmartDashboard.putString("Superstructure/CurrentShooterState", currentState.toString());
    Logger.recordOutput("Superstructure/PreviousShooterState", previousState);
    SmartDashboard.putString("Superstructure/PreviousShooterState", previousState.toString());
  }
}
