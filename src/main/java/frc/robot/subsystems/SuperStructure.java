package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.HopperRoller.HopperRoller.HopperRollerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.IntakeRoller.IntakeStateMachine;
import frc.robot.subsystems.IntakeRoller.IntakeStateMachine.IntakeWantedStates;
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
    UNJAMMING
  }

  public enum SuperInternalStates {
    DEFAULT, // flywheel spun up, hood prepping with ducking
    IDLE, // everything is stopped
    SHOOTING_AT_HUB,
    PASSING,
    UNJAMMING
  }

  // State variables
  private SuperWantedStates wantedSuperState = SuperWantedStates.DEFAULT;
  private SuperInternalStates currentSuperState = SuperInternalStates.DEFAULT;
  private SuperInternalStates previousSuperState = SuperInternalStates.DEFAULT;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

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
    this.shooterStateMachine =
        new ShooterStateMachine(
            flywheel, hood, flywheelKicker, isAlignedToTarget, this::canShootToTarget);
    this.intakeStateMachine = new IntakeStateMachine(intakeRoller, intakePivot);
    this.targetSelectionStateMachine =
        new TargetSelectionStateMachine(hubShotCalculator, passCalculator, robotPoseSupplier);

    flywheel.setShootVelocitySupplier(
        () -> targetSelectionStateMachine.getActiveCalculator().calculateShot().flywheelSpeed());
    hood.setHoodAngleSupplier(
        () -> targetSelectionStateMachine.getActiveCalculator().calculateShot().hoodAngle());
    hood.setShouldDuckSupplier(
        () -> PositionUtils.isInDuckZone(robotPoseSupplier.get(), robotToShooter));
  }

  // State machine methods

  private void updateState() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case SHOOT_AT_HUB:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.HUB);
        targetSelectionStateMachine.update();
        currentSuperState = SuperInternalStates.SHOOTING_AT_HUB;
        break;
      case SHOOT_AT_OUTPOST:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.OUTPOST);
        targetSelectionStateMachine.update();
        currentSuperState = SuperInternalStates.PASSING;
        break;
      case AUTO_CYCLE_SHOOTING:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.AUTO);
        targetSelectionStateMachine.update();
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
      case DEFAULT:
      default:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.AUTO);
        targetSelectionStateMachine.update();
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
      intakeStateMachine.setWantedState(IntakeWantedStates.AGITATING);
    } else {
      indexer.setWantedState(IndexerStates.OFF);
      hopperRoller.setWantedState(HopperRollerStates.OFF);
    }
  }

  private void passive_preparing() {
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
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
    switch (currentSuperState) {
      case SHOOTING_AT_HUB:
        if (!DriverStation.isFMSAttached()) {
          return true;
        }
        if (wantedSuperState == SuperWantedStates.SHOOT_AT_HUB) {
          return true;
        }
        boolean hubActive =
            RobotUtils.hubActive(
                DriverStation.getAlliance(),
                RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage()),
                RobotUtils.getShootingPhase(
                    DriverStation.getMatchTime(),
                    DriverStation.isTeleop(),
                    hubShotCalculator.calculateShot().timeOfFlight()));
        Logger.recordOutput("Superstructure/Shooting/HubActive", hubActive);
        return hubActive;
      default:
        return true;
    }
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
    return new InstantCommand(() -> setWantedSuperState(superState), this);
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

  public Command setIntakeStateCommand(IntakeStateMachine.IntakeWantedStates state) {
    return new InstantCommand(() -> setIntakeState(state), this);
  }

  // periodic

  @Override
  public void periodic() {
    // Runs the superstructure, shooter, and intake state machines
    updateState();
    shooterStateMachine.update();
    intakeStateMachine.update();

    applyStates();
    shooterStateMachine.apply();
    intakeStateMachine.apply();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);
    Logger.recordOutput("Superstructure/ControlState", controlState);
    shooterStateMachine.log();
    intakeStateMachine.log();
    targetSelectionStateMachine.log();
  }
}
