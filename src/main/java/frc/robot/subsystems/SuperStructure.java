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
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivot.IntakePivotWantedStates;
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
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private final Flywheel flywheel;
  private final Hood hood;
  private final IntakePivot intakePivot;
  private final HopperRoller hopperRoller;
  private final ShooterStateMachine shooterStateMachine;
  private final TargetSelectionStateMachine targetSelectionStateMachine;
  private final ShotCalculator hubShotCalculator;

  // Enums
  public enum SuperWantedStates {
    DEFAULT,
    IDLE,
    INTAKING,
    SHOOT_AT_HUB,
    SHOOT_AT_OUTPOST,
    AUTO_CYCLE_SHOOTING, // auto-selects hub or outpost based on alliance zone
    // TODO: not yet implemented
    DEFENSE,
    X_OUT,
    EJECTING
  }

  public enum SuperInternalStates {
    DEFAULT, // flywheel spun up, hood prepping with ducking
    IDLE, // everything is stopped
    INTAKING, // intake button pressed
    SHOOT_AT_HUB,
    SHOOT_PASSING
  }

  // State variables
  private SuperWantedStates wantedSuperState = SuperWantedStates.DEFAULT;
  private SuperInternalStates currentSuperState = SuperInternalStates.DEFAULT;
  private SuperInternalStates previousSuperState = SuperInternalStates.DEFAULT;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  public SuperStructure(
      Intake intake,
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
    this.intake = intake;
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
      case INTAKING:
        currentSuperState = SuperInternalStates.INTAKING;
        break;
      case SHOOT_AT_HUB:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.HUB);
        targetSelectionStateMachine.update();
        currentSuperState = SuperInternalStates.SHOOT_AT_HUB;
        break;
      case SHOOT_AT_OUTPOST:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.OUTPOST);
        targetSelectionStateMachine.update();
        currentSuperState = SuperInternalStates.SHOOT_PASSING;
        break;
      case AUTO_CYCLE_SHOOTING:
        targetSelectionStateMachine.setWantedState(TargetWantedStates.AUTO);
        targetSelectionStateMachine.update();
        if (targetSelectionStateMachine.getState() == TargetInternalStates.AT_HUB) {
          currentSuperState = SuperInternalStates.SHOOT_AT_HUB;
        } else {
          currentSuperState = SuperInternalStates.SHOOT_PASSING;
        }
        break;
      case IDLE:
        currentSuperState = SuperInternalStates.IDLE;
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
      case INTAKING:
        intaking();
        break;
      case IDLE:
        stopped();
        break;
      case SHOOT_AT_HUB:
      case SHOOT_PASSING:
        shooting();
        break;
      case DEFAULT:
        passivePrep();
        break;
    }
  }

  // Subsystem state helpers

  private void shooting() {
    shooterStateMachine.setWantedState(ShooterWantedStates.SHOOTING);

    if (shooterStateMachine.getState() == ShooterStates.FIRING) {
      indexer.setWantedState(IndexerStates.SHOOTING);
      hopperRoller.setWantedState(HopperRollerStates.ROLLING);
      intakePivot.setWantedState(IntakePivotWantedStates.AGITATE_HOPPER);
    } else {
      indexer.setWantedState(IndexerStates.OFF);
      hopperRoller.setWantedState(HopperRollerStates.OFF);
    }
  }

  private void passivePrep() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    intakePivot.setWantedState(IntakePivotWantedStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
    shooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_SHOOTER);
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    intakePivot.setWantedState(IntakePivotWantedStates.DEPLOYED);
    shooterStateMachine.setWantedState(ShooterWantedStates.IDLE);
    // indexer.setWantedState(Indexer.IndexerStates.INTAKING);
  }

  private void stopped() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    intakePivot.setWantedState(IntakePivotWantedStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
    shooterStateMachine.setWantedState(ShooterWantedStates.IDLE);
  }

  private boolean canShootToTarget() {
    switch (wantedSuperState) {
      case SHOOT_AT_HUB:
        if (!DriverStation.isDSAttached()) {
          return true;
        }
        return RobotUtils.hubActive(
            DriverStation.getAlliance(),
            RobotUtils.getAutoWinner(DriverStation.getGameSpecificMessage()),
            RobotUtils.getShootingPhase(
                DriverStation.getMatchTime(),
                DriverStation.isTeleop(),
                hubShotCalculator.calculateShot().timeOfFlight()));
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
    intake.setControlState(controlState);
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

  // periodic

  @Override
  public void periodic() {
    // Runs both the superstructure and shooter state machines
    updateState();
    shooterStateMachine.update();

    applyStates();
    shooterStateMachine.apply();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);
    Logger.recordOutput("Superstructure/ControlState", controlState);
    shooterStateMachine.log();
    targetSelectionStateMachine.log();
  }
}
