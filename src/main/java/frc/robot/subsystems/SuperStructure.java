package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelWantedStates;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.ShotCalculator;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private final Flywheel flywheel;
  private final Hood hood;
  private final ShotCalculator hubShotCalculator;
  private final ShotCalculator outpostPassCalculator;
  private final BooleanSupplier isAlignedToTarget;

  public enum SuperStates {
    IDLE, // everything is stopped when nothing else happens
    INTAKING, // while intake button pressed
    SHOOT_AT_HUB,
    // TODO: not yet implemented
    DEFENSE, // driver holds defense button -> less desired velocity moving laterally, more rotation
    X_OUT, // hold down button to x out wheels or press once and wheels stop X-ing out when moved
    AUTO_ALIGN, // aligns to a target
    X_OUT_SHOOTING, // when robot is aligned, ends when toggled off or shooting stops
    FIRING, // while there's still fuel to shoot and ready to fire
    EJECTING, // eject button
    SHOOT_AT_OUTPOST // has current zone, makes check for !current zone then passes to zone
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
      ShotCalculator hubShotCalculator,
      ShotCalculator outpostPassCalculator,
      BooleanSupplier isAlignedToTarget) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.hubShotCalculator = hubShotCalculator;
    this.outpostPassCalculator = outpostPassCalculator;
    this.isAlignedToTarget = isAlignedToTarget;

    flywheel.setShootVelocitySupplier(
        () -> {
          if (currentSuperState == SuperStates.SHOOT_AT_OUTPOST) {
            return this.outpostPassCalculator.calculateShot().flywheelSpeed();
          }
          return this.hubShotCalculator.calculateShot().flywheelSpeed();
        });
    // hood.setHoodAngleSupplier(
    //     () -> {
    //       if (currentSuperState == SuperStates.SHOOT_AT_OUTPOST) {
    //         return this.outpostPassCalculator.calculateShot().hoodAngle();
    //       }
    //       return this.hubShotCalculator.calculateShot().hoodAngle();
    //     });
  }

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
      case SHOOT_AT_HUB:
      case SHOOT_AT_OUTPOST:
        updateShooterStates();
        applyShooterStates();
        break;
    }
    resetShooterStateIfNotShooting();
  }

  private void resetShooterStateIfNotShooting() {
    if (currentSuperState != SuperStates.SHOOT_AT_HUB
        && currentSuperState != SuperStates.SHOOT_AT_OUTPOST) {
      currentShooterState = ShooterStates.PREPARING;
    }
  }

  private void updateShooterStates() {
    previousShooterState = currentShooterState;
    boolean flywheelReady = flywheel.getState() == FlywheelStates.AT_SETPOINT;
    // boolean hoodReady = hood.getState() == HoodStates.AT_SETPOINT;
    boolean aligned = isAlignedToTarget.getAsBoolean();

    Logger.recordOutput("Superstructure/Shooting/FlywheelReady", flywheelReady);
    // Logger.recordOutput("Superstructure/Shooting/HoodReady", hoodReady);
    Logger.recordOutput("Superstructure/Shooting/Aligned", aligned);

    if (flywheelReady && aligned) { // ADD HOOD CHECK BACK WHEN GOOD AGAIN
      currentShooterState = ShooterStates.FIRING;
    } else {
      currentShooterState = ShooterStates.PREPARING;
    }
  }

  private void applyShooterStates() {
    aiming();
    if (currentShooterState == ShooterStates.FIRING) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SHOOTING);
      indexer.setWantedState(IndexerStates.SHOOTING);
    } else {
      flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
      indexer.setWantedState(IndexerStates.OFF);
    }
  }

  private void aiming() {
    flywheel.setWantedState(FlywheelWantedStates.AIMING);
    // hood.setWantedState(HoodWantedStates.AIMING);
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    // indexer.setWantedState(Indexer.IndexerStates.INTAKING);
  }

  private void stopped() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
    flywheel.setWantedState(FlywheelWantedStates.IDLE);
    // hood.setWantedState(HoodWantedStates.IDLE);
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
