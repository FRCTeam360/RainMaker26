package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodStates;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private Flywheel flywheel;
  private Hood hood;

  public enum SuperStates {
    IDLE, // everything is stopped when nothing else happens
    DEFENSE, // driver holds defense button -> less desired velocitu moving latterally, more
    // into
    // rotation in drivetrain
    X_OUT, // hold down button to x out wheels or press once and wheels stop X-ing out when
    // moved
    AUTO_ALIGHN, // aligns to a target
    X_OUT_SHOOTING, // when robot is aligned, ends when toggled off or shooting stops
    PREPAREING, // flywheel spins up and align to target
    READY_2_FIRE, // if robot aligned and flywheel up to proper speed
    FIRING, // while theres still fuel to shoot and ready to fire
    INTAKING, // while intake button pressed
    EJECTING // eject button
    ,
    PASSING // has current zone, makes check for !current zone then passes to zone
    ,
    SHOOTING,
    SPINUP_SHOOTING
  }

  private SuperStates wantedSuperState = SuperStates.IDLE;
  private SuperStates currentSuperState = SuperStates.IDLE;
  private SuperStates previousSuperState = SuperStates.IDLE;

  public SuperStructure(
      Intake intake, Indexer indexer, FlywheelKicker flywheelKicker, Flywheel flywheel, Hood hood) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
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
        currentSuperState = SuperStates.IDLE;
        break;
      case SPINUP_SHOOTING:
        currentSuperState = SuperStates.SPINUP_SHOOTING;
        break;
    }
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKING:
        intaking();
        break;
      case SHOOTING:
        shooting();
        break;

      case IDLE:
        stopped();
        break;
      case SPINUP_SHOOTING:
        spinupShooting();
        break;
    }
  }

  private void spinupShooting() {
    // hood, flywheel
    hood.setWantedState(HoodStates.SPINUP_SHOOTING);
    flywheel.setWantedState(FlywheelStates.SPINUP_SHOOTING);
    if (hood.atSetpoint(8.0) && flywheel.atSetpoint(3500.0, 100.0)) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SPINUP_SHOOTING);
      intake.setWantedState(IntakeStates.INTAKING);
      indexer.setWantedState(IndexerStates.SHOOTING);
    }
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    indexer.setWantedState(Indexer.IndexerStates.INTAKING);
  }

  private void shooting() {
    flywheel.setWantedState(Flywheel.FlywheelStates.SHOOTING);
  }

  private void stopped() {
    intake.setWantedState(Intake.IntakeStates.OFF);
    indexer.setWantedState(Indexer.IndexerStates.OFF);
    flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
    flywheel.setWantedState(FlywheelStates.OFF);
    hood.setWantedState(HoodStates.OFF);
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
  }
}
