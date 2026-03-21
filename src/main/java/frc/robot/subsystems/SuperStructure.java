package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.HopperRoller.HopperRoller.HopperRollerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakePivot.IntakePivot;
import frc.robot.subsystems.Intake.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.Intake.IntakeStateMachine;
import frc.robot.subsystems.Intake.IntakeStateMachine.IntakeWantedStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterStates;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterWantedStates;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine.TargetInternalStates;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine.TargetWantedStates;
import frc.robot.utils.PositionUtils;
import frc.robot.utils.RobotUtils;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  // Fields (subsystem refs, calculators, suppliers)
  private final IntakeRoller intakeRoller;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private final Flywheel flywheel;
  private final Hood hood;
  private final IntakePivot intakePivot;
  private final HopperRoller hopperRoller;
  private final ShooterStateMachine shooterStateMachine;
  private final IntakeStateMachine intakeStateMachine;
  private final TargetSelectionStateMachine targetSelectionStateMachine;
  private final ShotCalculator hubShotCalculator;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Transform2d robotToShooter;

  // Enums
  public enum SuperWantedStates {
    DEFAULT,
    IDLE,
    SHOOT_AT_HUB,
    SHOOT_AT_OUTPOST,
    AUTO_CYCLE_SHOOTING, // auto-selects hub or outpost based on alliance zone
    // TODO: not yet implemented
    DEFENSE,
    X_OUT,
    EJECTING,
    UNJAMMING,
    FORCED_SHOT
  }

  public enum SuperInternalStates {
    DEFAULT, // flywheel spun up, hood prepping with ducking
    IDLE, // everything is stopped
    SHOOTING_AT_HUB,
    PASSING,
    UNJAMMING,
    FORCED_SHOT
  }

  // State variables
  private SuperWantedStates wantedSuperState = SuperWantedStates.IDLE;
  private SuperInternalStates currentSuperState = SuperInternalStates.IDLE;
  private SuperInternalStates previousSuperState = SuperInternalStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;
  private boolean cachedHubActive = true;
  private double cachedTimeOfFlight = 0.0;

  // Constructor

  public SuperStructure(
      IntakeRoller intakeRoller,
      Indexer indexer,
      FlywheelKicker flywheelKicker,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      HopperRoller hopperRoller,
      ShotCalculator hubShotCalculator,
      ShotCalculator passCalculator,
      BooleanSupplier isAlignedToTarget,
      Supplier<Pose2d> robotPoseSupplier,
      Transform2d robotToShooter) {
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.intakePivot = intakePivot;
    this.hopperRoller = hopperRoller;
    this.hubShotCalculator = hubShotCalculator;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotToShooter = robotToShooter;
    this.shooterStateMachine =
        new ShooterStateMachine(
            flywheel, hood, flywheelKicker, isAlignedToTarget, this::canShootToTarget);
    this.intakeStateMachine = new IntakeStateMachine(intakeRoller, intakePivot);
    this.targetSelectionStateMachine =
        new TargetSelectionStateMachine(hubShotCalculator, passCalculator, robotPoseSupplier);

    flywheel.setShootVelocitySupplier(
        () -> {
          if (currentSuperState == SuperInternalStates.FORCED_SHOT) {
            return 2000;
          }
          return targetSelectionStateMachine.getActiveCalculator().calculateShot().flywheelSpeed();
          // () -> targetSelectionStateMachine.getActiveCalculator().calculateShot().flywheelSpeed()
        });
    hood.setHoodAngleSupplier(
        () -> {
          if (currentSuperState == SuperInternalStates.FORCED_SHOT) {
            return 20.0;
          }
          return targetSelectionStateMachine.getActiveCalculator().calculateShot().hoodAngle();
        });
    hood.setShouldDuckSupplier(
        () -> PositionUtils.isInDuckZone(robotPoseSupplier.get(), robotToShooter));
  }

  // State machine methods

  private void updateState() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case SHOOT_AT_HUB:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.HUB);
        currentSuperState = SuperInternalStates.SHOOTING_AT_HUB;
        break;
      case SHOOT_AT_OUTPOST:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.OUTPOST);
        currentSuperState = SuperInternalStates.PASSING;
        break;
      case AUTO_CYCLE_SHOOTING:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.AUTO);
        if (targetSelectionStateMachine.getState() == TargetInternalStates.AT_HUB) {
          currentSuperState = SuperInternalStates.SHOOTING_AT_HUB;
        } else {
          currentSuperState = SuperInternalStates.PASSING;
        }
        break;
      case IDLE:
        currentSuperState = SuperInternalStates.IDLE;
        break;
      case UNJAMMING:
        currentSuperState = SuperInternalStates.UNJAMMING;
        break;
      case FORCED_SHOT:
        currentSuperState = SuperInternalStates.FORCED_SHOT;
      case DEFAULT:
      default:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.AUTO);
        currentSuperState = SuperInternalStates.DEFAULT;
        break;
    }
  }

  private void applyStates() {
    switch (currentSuperState) {
      case IDLE:
        stopped();
        break;
      case SHOOTING_AT_HUB:
      case PASSING:
        shooting();
        break;
      case UNJAMMING:
        unjamming();
        break;
      case FORCED_SHOT:
        shooting();
        shooterStateMachine.setWantedState(ShooterWantedStates.FORCED_SHOT);
        break;
      case DEFAULT:
        passive_preparing();
        break;
    }
  }

  // Subsystem state helpers

  private void shooting() {
    shooterStateMachine.setWantedState(ShooterWantedStates.SHOOTING);

    if (shooterStateMachine.getState() == ShooterStates.FIRING) {
      indexer.setWantedState(IndexerStates.INDEXING);
      hopperRoller.setWantedState(HopperRollerStates.ROLLING);
    } else {
      indexer.setWantedState(IndexerStates.OFF);
      hopperRoller.setWantedState(HopperRollerStates.PREVENT_JAM);
    }
  }

  private void passive_preparing() {
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.PREVENT_JAM);
    shooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_SHOOTER);
  }

  private void stopped() {
    intakeStateMachine.setWantedState(IntakeWantedStates.IDLE);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
    shooterStateMachine.setWantedState(ShooterWantedStates.IDLE);
  }

  private void unjamming() {
    indexer.setWantedState(IndexerStates.REVERSING);
    shooterStateMachine.setWantedState(ShooterWantedStates.REVERSING);
    hopperRoller.setWantedState(HopperRollerStates.UNJAMMING);
  }

  private boolean canShootToTarget() {
    if (wantedSuperState == SuperWantedStates.AUTO_CYCLE_SHOOTING) {
      switch (currentSuperState) {
        case SHOOTING_AT_HUB:
          if (!DriverStation.isFMSAttached()) {
            return true;
          }
          // Allow shooting if explicitly commanded to shoot at hub (manual override)
          if (wantedSuperState == SuperWantedStates.SHOOT_AT_HUB) {
            return true;
          }
          // For AUTO_CYCLE_SHOOTING, check if hub is actually active based on game phase
          return canScoreAtHub() && hubShotCalculator.calculateShot().isValid();
        case PASSING:
          boolean isInPassingZone =
              PositionUtils.isInPassingZone(robotPoseSupplier.get(), robotToShooter);
          return isInPassingZone;
        default:
          return true;
      }
    }
    return true;
  }

  // Public API

  /** Sets the control mode and propagates it to managed subsystems. */
  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
    flywheel.setControlState(controlState);
    indexer.setControlState(controlState);
    flywheelKicker.setControlState(controlState);
    intakeRoller.setControlState(controlState);
    intakePivot.setControlState(controlState);
    hopperRoller.setControlState(controlState);
    hood.setControlState(controlState);
  }

  /** Returns the current control mode. */
  public ControlState getControlState() {
    return controlState;
  }

  public Command setStateCommand(SuperWantedStates superState) {
    return new InstantCommand(() -> setWantedSuperState(superState));
  }

  public void setWantedSuperState(SuperWantedStates superState) {
    this.wantedSuperState = superState;
  }

  /** Returns the current (resolved) super state. */
  public SuperInternalStates getCurrentSuperState() {
    return currentSuperState;
  }

  /**
   * Sets the intake state machine's wanted state directly.
   *
   * @param state the desired intake state
   */
  public void setIntakeState(IntakeStateMachine.IntakeWantedStates state) {
    intakeStateMachine.setWantedState(state);
  }

  /** Returns the current intake internal state. */
  public IntakeStateMachine.IntakeInternalStates getIntakeState() {
    return intakeStateMachine.getState();
  }

  public Command setIntakeStateCommand(IntakeStateMachine.IntakeWantedStates state) {
    return new InstantCommand(() -> setIntakeState(state));
  }

  /**
   * Returns a command that toggles the intake between STOWED and INTAKING. If the intake is
   * currently STOWED, it switches to INTAKING; otherwise it switches to STOWED.
   */
  public Command toggleIntakeStateCommand() {
    return new InstantCommand(
        () -> {
          if (intakeStateMachine.getState() == IntakeStateMachine.IntakeInternalStates.STOWED) {
            intakeStateMachine.setWantedState(IntakeWantedStates.INTAKING);
          } else {
            intakeStateMachine.setWantedState(IntakeWantedStates.STOWED);
          }
        });
  }

  /**
   * Returns whether the hub is currently active/shootable based on game time, alliance, and auto
   * winner. Does not require the superstructure to be in SHOOT_AT_HUB state.
   *
   * @return true if the hub is active and can be shot at, false otherwise.
   */
  public boolean canScoreAtHub() {
    return cachedHubActive;
  }

  // periodic

  @Override
  public void periodic() {
    // Calculate shot and extract time of flight once per cycle
    cachedTimeOfFlight = hubShotCalculator.calculateShot().timeOfFlight();
    RobotUtils.ActiveHub shootingPhase =
        RobotUtils.getShootingPhase(
            DriverStation.getMatchTime(), DriverStation.isTeleop(), cachedTimeOfFlight);

    // Calculate hub active once per cycle
    cachedHubActive =
        RobotUtils.hubActive(
            DriverStation.getAlliance(),
            RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage()),
            shootingPhase);

    // Runs the superstructure, shooter, and intake state machines
    updateState();
    targetSelectionStateMachine.update();
    shooterStateMachine.update();
    intakeStateMachine.update();

    applyStates();
    shooterStateMachine.apply();
    intakeStateMachine.apply();

    SmartDashboard.putString("Shooting Phase", shootingPhase.toString());
    SmartDashboard.putBoolean("Can Score in Hub", cachedHubActive);

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    SmartDashboard.putString("Superstructure/CurrentSuperState", currentSuperState.toString());
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);
    Logger.recordOutput("Superstructure/ControlState", controlState);
    Logger.recordOutput("Superstructure/HubActive", cachedHubActive);
    shooterStateMachine.log();
    intakeStateMachine.log();
    targetSelectionStateMachine.log();
  }
}
