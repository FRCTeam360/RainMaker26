package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
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
 * Manages the shooter state machine, coordinating the flywheel, hood, and
 * flywheel kicker to
 * determine when the shooter is ready to fire.
 */
public class ShooterStateMachine {

  private static final double DISTURBANCE_TIMEOUT_SECONDS = 0.5;

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
    DISTURBED,
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
  private double disturbanceStartTimestampSeconds = Double.NaN;

  /**
   * Creates a new ShooterStateMachine.
   *
   * @param flywheel          the flywheel subsystem
   * @param hood              the hood subsystem
   * @param flywheelKicker    the flywheel kicker subsystem
   * @param isAlignedToTarget supplier that returns true when the robot is aligned
   *                          to the target
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
   * Updates the shooter state based on wanted state, subsystem readiness, and
   * alignment. Should be
   * called every cycle by the SuperStructure.
   *
   * <p>
   * Uses the flywheel's internal state transitions to gate firing:
   *
   * <ul>
   * <li>{@link FlywheelInternalStates#AT_SETPOINT} — flywheel velocity is
   * sustained in tolerance;
   * combined with hood and drivetrain readiness, transitions to FIRING
   * <li>{@link FlywheelInternalStates#UNDER_SHOOTING} — sustained RPM drop
   * detected from a shot
   * passing through; reverts to PREPARING_TO_FIRE to restart the cycle
   * </ul>
   */
  public void update() {
    previousState = currentState;

    switch (wantedState) {
      case FORCED_SHOT:
        handleShooting(true);
        break;
      case SHOOTING:
        handleShooting(false);
        break;
      case PASSIVE_SHOOTER:
        currentState = isInAllianceZone.getAsBoolean() ? ShooterStates.STANDBY : ShooterStates.WAITING;
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
   * Handles the shooting state logic, determining readiness and transitioning
   * states.
   *
   * @param isForced if true, bypasses drivetrain alignment and target readiness
   *                 checks
   */
  private void handleShooting(boolean isForced) {
    FlywheelInternalStates flywheelState = flywheel.getState();
    boolean flywheelReady = flywheelState == FlywheelInternalStates.AT_SETPOINT;
    boolean flywheelUnderShooting = flywheelState == FlywheelInternalStates.UNDER_SHOOTING;
    boolean hoodReady = hood.getState() == HoodInternalStates.AT_SETPOINT;
    boolean drivetrainAligned = isForced || isAlignedToTarget.getAsBoolean();
    boolean targetReady = isForced || canShootToTarget.getAsBoolean();
    boolean hoodAndDriveReady = hoodReady && drivetrainAligned;
    boolean wasFiringOrDisturbed = previousState == ShooterStates.FIRING || previousState == ShooterStates.DISTURBED;
    boolean disturbanceActive = wasFiringOrDisturbed && !hoodAndDriveReady;

    Logger.recordOutput("Superstructure/Shooter/Shooting/FlywheelReady", flywheelReady);
    SmartDashboard.putBoolean("Superstructure/Shooter/Shooting/FlywheelReady", flywheelReady);
    Logger.recordOutput("Superstructure/Shooter/Shooting/HoodReady", hoodReady);
    SmartDashboard.putBoolean("Superstructure/Shooter/Shooting/HoodReady", hoodReady);
    Logger.recordOutput("Superstructure/Shooter/Shooting/DrivetrainAligned", drivetrainAligned);
    Logger.recordOutput("Superstructure/Shooter/Shooting/TargetReady", targetReady);
    SmartDashboard.putBoolean(
        "Superstructure/Shooter/Shooting/DrivetrainAligned", drivetrainAligned);

    // AIMED requires flywheel and hood to be genuinely at setpoint.
    // Bang-bang oscillations during firing are tolerated to stay in FIRING —
    // only revert when UNDER_SHOOTING signals a sustained RPM drop.
    boolean inBangBang = wasFiringOrDisturbed && !flywheelUnderShooting;
    boolean subsystemsReady = flywheelReady && hoodAndDriveReady;
    boolean shouldFire = (flywheelReady || inBangBang) && hoodAndDriveReady && targetReady;

    Logger.recordOutput("Superstructure/Shooter/Shooting/InBangBang", inBangBang);
    Logger.recordOutput("Superstructure/Shooter/Shooting/SubsystemsReady", subsystemsReady);
    Logger.recordOutput("Superstructure/Shooter/Shooting/ShouldFire", shouldFire);
    Logger.recordOutput("Superstructure/Shooter/Shooting/DisturbanceActive", false);
    Logger.recordOutput("Superstructure/Shooter/Shooting/DisturbanceTimedOut", false);
    Logger.recordOutput(
        "Superstructure/Shooter/Shooting/DisturbanceWindowSec", DISTURBANCE_TIMEOUT_SECONDS);

    if (disturbanceActive) {
      if (previousState != ShooterStates.DISTURBED
          || Double.isNaN(disturbanceStartTimestampSeconds)) {
        disturbanceStartTimestampSeconds = Timer.getFPGATimestamp();
      }

      boolean disturbanceTimedOut = Timer.getFPGATimestamp()
          - disturbanceStartTimestampSeconds >= DISTURBANCE_TIMEOUT_SECONDS;

      Logger.recordOutput(
          "Superstructure/Shooter/Shooting/DisturbanceTimedOut", disturbanceTimedOut);
      Logger.recordOutput(
          "Superstructure/Shooter/Shooting/DisturbanceWindowSec", DISTURBANCE_TIMEOUT_SECONDS);

      if (disturbanceTimedOut) {
        disturbanceStartTimestampSeconds = Double.NaN;
        currentState = ShooterStates.PREPARING_TO_FIRE;
      } else {
        currentState = ShooterStates.DISTURBED;
      }

      return;
    }

    disturbanceStartTimestampSeconds = Double.NaN;

    if (shouldFire) {
      currentState = ShooterStates.FIRING;
    } else if (subsystemsReady) {
      currentState = ShooterStates.AIMED;
    } else {
      currentState = ShooterStates.PREPARING_TO_FIRE;
    }
  }

  /**
   * Applies the current shooter state to subsystems. Sets flywheel, hood, and
   * flywheel kicker
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
      case DISTURBED:
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
    Logger.recordOutput("Superstructure/Shooter/WantedState", wantedState);
    SmartDashboard.putString("Superstructure/Shooter/WantedState", wantedState.toString());
    Logger.recordOutput("Superstructure/Shooter/CurrentState", currentState);
    SmartDashboard.putString("Superstructure/Shooter/CurrentState", currentState.toString());
    Logger.recordOutput("Superstructure/Shooter/PreviousState", previousState);
    SmartDashboard.putString("Superstructure/Shooter/PreviousState", previousState.toString());
  }
}
