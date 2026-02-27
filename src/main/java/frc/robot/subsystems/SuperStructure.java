package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.HopperRoller.HopperRoller.HopperRollerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivot.IntakePivotStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterStates;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterWantedStates;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.utils.PositionUtils;
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
  private final ShotCalculator hubShotCalculator;
  private final ShotCalculator outpostPassCalculator;
  private final ShooterStateMachine shooterStateMachine;
  private final Supplier<Pose2d> robotPoseSupplier;

  // Enums
  public enum SuperStates {
    PASSIVE_PREP, // default state: flywheel spun up, hood prepping with ducking
    IDLE, // everything is stopped when nothing else happens
    INTAKING, // while intake button pressed
    SHOOT_AT_HUB,
    // TODO: not yet implemented
    DEFENSE, // driver holds defense button -> less desired velocity moving laterally, more rotation
    X_OUT, // hold down button to x out wheels or press once and wheels stop X-ing out when moved
    X_OUT_SHOOTING, // when robot is aligned, ends when toggled off or shooting stops
    FIRING, // while there's still fuel to shoot and ready to fire
    EJECTING, // eject button
    SHOOT_AT_OUTPOST, // has current zone, makes check for !current zone then passes to zone
    AUTO_CYCLE_SHOOTING // auto-selects SHOOT_AT_HUB or SHOOT_AT_OUTPOST based on alliance zone
  }

  // State variables
  private SuperStates wantedSuperState = SuperStates.PASSIVE_PREP;
  private SuperStates currentSuperState = SuperStates.PASSIVE_PREP;
  private SuperStates previousSuperState = SuperStates.PASSIVE_PREP;
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
      ShotCalculator outpostPassCalculator,
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
    this.outpostPassCalculator = outpostPassCalculator;
    this.robotPoseSupplier = robotPoseSupplier;
    this.shooterStateMachine =
        new ShooterStateMachine(flywheel, hood, flywheelKicker, isAlignedToTarget);

    flywheel.setShootVelocitySupplier(() -> getActiveCalculator().calculateShot().flywheelSpeed());
    hood.setHoodAngleSupplier(() -> getActiveCalculator().calculateShot().hoodAngle());
    hood.setShouldDuckSupplier(
        () -> PositionUtils.isInDuckZone(robotPoseSupplier.get(), robotToShooter));
  }

  // State machine methods

  private void updateState() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case INTAKING:
        currentSuperState = SuperStates.INTAKING;
        break;
      case SHOOT_AT_HUB:
        currentSuperState = SuperStates.SHOOT_AT_HUB;
        break;
      case SHOOT_AT_OUTPOST:
        currentSuperState = SuperStates.SHOOT_AT_OUTPOST;
        break;
      case AUTO_CYCLE_SHOOTING:
        if (PositionUtils.isInAllianceZone(robotPoseSupplier.get())) {
          currentSuperState = SuperStates.SHOOT_AT_HUB;
        } else {
          currentSuperState = SuperStates.SHOOT_AT_OUTPOST;
        }
        break;
      case IDLE:
        currentSuperState = SuperStates.IDLE;
        break;
      case PASSIVE_PREP:
      default:
        currentSuperState = SuperStates.PASSIVE_PREP;
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
      case SHOOT_AT_OUTPOST:
        shooting();
        break;
      case PASSIVE_PREP:
        passivePrep();
        break;
    }
  }

  /**
   * Returns the shot calculator to use for flywheel/hood setpoints based on the current state.
   *
   * <p>Uses the outpost calculator when actively shooting at the outpost, or during passive prep
   * when the robot is outside its alliance zone. Otherwise uses the hub calculator.
   */
  private ShotCalculator getActiveCalculator() {
    if (currentSuperState == SuperStates.SHOOT_AT_OUTPOST) {
      return outpostPassCalculator;
    }
    if (currentSuperState == SuperStates.PASSIVE_PREP
        && !PositionUtils.isInAllianceZone(robotPoseSupplier.get())) {
      return outpostPassCalculator;
    }
    return hubShotCalculator;
  }

  // Subsystem state helpers

  private void shooting() {
    shooterStateMachine.setWantedState(ShooterWantedStates.SHOOTING);

    if (shooterStateMachine.getState() == ShooterStates.FIRING) {
      indexer.setWantedState(IndexerStates.SHOOTING);
      hopperRoller.setWantedState(HopperRollerStates.ROLLING);
    } else {
      indexer.setWantedState(IndexerStates.OFF);
      hopperRoller.setWantedState(HopperRollerStates.OFF);
    }
  }

  private void passivePrep() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    intakePivot.setWantedState(IntakePivotStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
    shooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_PREP);
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    intakePivot.setWantedState(IntakePivotStates.DEPLOYED);
    shooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_PREP);
    // indexer.setWantedState(Indexer.IndexerStates.INTAKING);
  }

  private void stopped() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    intakePivot.setWantedState(IntakePivotStates.OFF);
    hopperRoller.setWantedState(HopperRollerStates.OFF);
    shooterStateMachine.setWantedState(ShooterWantedStates.IDLE);
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

  public Command setStateCommand(SuperStates superState) {
    return new InstantCommand(() -> setWantedSuperState(superState), this);
  }

  public void setWantedSuperState(SuperStates superState) {
    this.wantedSuperState = superState;
  }

  /** Returns the current (resolved) super state. */
  public SuperStates getCurrentSuperState() {
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
  }
}
