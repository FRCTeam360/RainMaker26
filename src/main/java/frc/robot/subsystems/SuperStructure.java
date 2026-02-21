package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.HopperRoller.HopperRoller.HopperRollerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivot.IntakePivotStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParams;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private final Flywheel flywheel;
  private final Hood hood;
  private final IntakePivot intakePivot;
  private final HopperRoller hopperRoller;
  private final ShotCalculator shotCalculator;
  private final BooleanSupplier isAlignedToTarget;

  public enum SuperStates {
    IDLE, // everything is stopped when nothing else happens
    INTAKING, // while intake button pressed
    SHOOTING,
    // TODO: not yet implemented
    DEFENSE, // driver holds defense button -> less desired velocity moving laterally, more rotation
    X_OUT, // hold down button to x out wheels or press once and wheels stop X-ing out when moved
    AUTO_ALIGN, // aligns to a target
    X_OUT_SHOOTING, // when robot is aligned, ends when toggled off or shooting stops
    FIRING, // while there's still fuel to shoot and ready to fire
    EJECTING, // eject button
    PASSING // has current zone, makes check for !current zone then passes to zone
  }

  public enum ShooterStates {
    FIRING,
    PREPARING
  }

  private SuperStates wantedSuperState = SuperStates.IDLE;
  private SuperStates currentSuperState = SuperStates.IDLE;
  private SuperStates previousSuperState = SuperStates.IDLE;

  private ShooterStates currentShooterState = ShooterStates.PREPARING;
  private ShooterStates previousShooterState = ShooterStates.PREPARING;

  public SuperStructure(
      Intake intake,
      Indexer indexer,
      FlywheelKicker flywheelKicker,
      Flywheel flywheel,
      Hood hood,
      IntakePivot intakePivot,
      HopperRoller hopperRoller,
      ShotCalculator shotCalculator,
      BooleanSupplier isAlignedToTarget) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.intakePivot = intakePivot;
    this.hopperRoller = hopperRoller;
    this.shotCalculator = shotCalculator;
    this.isAlignedToTarget = isAlignedToTarget;

    flywheel.setVelocitySupplier(() -> shotCalculator.calculateShot().flywheelSpeed());
    // hood.setHoodAngleSupplier(() -> shotCalculator.calculateShot().hoodAngle()); FIX ME WHEN
    // BETTER HOOD
  }

  private void updateState() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case INTAKING:
        currentSuperState = SuperStates.INTAKING;
        break;
      case SHOOTING:
        currentSuperState = SuperStates.SHOOTING;
        break;
      case IDLE:
      default:
        currentSuperState = SuperStates.IDLE;
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
      case SHOOTING:
        updateShooterStates();
        applyShooterStates();
        break;
    }
    resetShooterStateIfNotShooting();
  }

  private static final double FLYWHEEL_TOLERANCE_RPM = 500.0;

  private void resetShooterStateIfNotShooting() {
    if (currentSuperState != SuperStates.SHOOTING) {
      currentShooterState = ShooterStates.PREPARING;
    }
  }

  private void updateShooterStates() {
    previousShooterState = currentShooterState;
    ShootingParams shotParams = shotCalculator.calculateShot();
    boolean flywheelReady = flywheel.atSetpoint(shotParams.flywheelSpeed(), FLYWHEEL_TOLERANCE_RPM);
    // boolean hoodReady = hood.atSetpoint(shotParams.hoodAngle()); FIX ME WHEN BETTER HOOD
    boolean aligned = isAlignedToTarget.getAsBoolean();

    Logger.recordOutput("Superstructure/Shooting/FlywheelReady", flywheelReady);
    // Logger.recordOutput("Superstructure/Shooting/HoodReady", hoodReady); FIX ME WHEN BETTER HOOD
    Logger.recordOutput("Superstructure/Shooting/Aligned", aligned);

    if (flywheelReady && aligned) { // ADD HOOD CHECK WHEN BETTER HOOD
      currentShooterState = ShooterStates.FIRING;
    } else {
      currentShooterState = ShooterStates.PREPARING;
    }
  }

  private void applyShooterStates() {
    shooting();
    if (currentShooterState == ShooterStates.FIRING) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SHOOTING);
      indexer.setWantedState(IndexerStates.SHOOTING);
      hopperRoller.setWantedState(HopperRollerStates.ROLLING);
    } else {
      flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
      indexer.setWantedState(IndexerStates.OFF);
      hopperRoller.setWantedState(HopperRollerStates.OFF);
    }
  }

  private void shooting() {
    flywheel.setWantedState(FlywheelStates.SHOOTING);
    // hood.setWantedState(HoodStates.SHOOTING); FIX ME WHEN BETTER HOOD
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    intakePivot.setWantedState(IntakePivotStates.DEPLOYED);
    // indexer.setWantedState(Indexer.IndexerStates.INTAKING);
  }

  private void stopped() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
    flywheel.setWantedState(FlywheelStates.OFF);
    // hood.setWantedState(HoodStates.OFF); FIX ME WHEN BETTER HOOD
    intakePivot.setWantedState(IntakePivotStates.OFF);
  }

  public Command setStateCommand(SuperStates superState) {
    return new InstantCommand(() -> setWantedSuperState(superState), this);
  }

  public void setWantedSuperState(SuperStates superState) {
    this.wantedSuperState = superState;
  }

  @Override
  public void periodic() {
    updateState();
    applyStates();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState.toString());
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState.toString());
    Logger.recordOutput("Superstructure/PreviousShooterState", previousShooterState.toString());
    Logger.recordOutput("Superstructure/CurrentShooterState", currentShooterState.toString());
  }
}
