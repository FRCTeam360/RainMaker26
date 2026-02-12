package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
import frc.robot.subsystems.Shooter.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private Flywheel flywheel;
  private Hood hood;
  private CommandSwerveDrivetrain drivetrain;
  private ShotCalculator shotCalculator;

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
    SPINUP_SHOOTING,
    AIMING
  }

  private SuperStates wantedSuperState = SuperStates.IDLE;
  private SuperStates currentSuperState = SuperStates.IDLE;
  private SuperStates previousSuperState = SuperStates.IDLE;

  public SuperStructure(
      Intake intake,
      Indexer indexer,
      FlywheelKicker flywheelKicker,
      Flywheel flywheel,
      Hood hood,
      CommandSwerveDrivetrain driveTrain,
      ShotCalculator shotCalculator) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.drivetrain = driveTrain;
    this.shotCalculator = shotCalculator;
    hood.setHoodAngleSupplier(() -> shotCalculator.calculateShot().hoodAngle());
  }

  private void updateState() {
    previousSuperState = currentSuperState;

    switch (wantedSuperState) {
      case INTAKING:
        currentSuperState = SuperStates.INTAKING;
        break;
      case SPINUP_SHOOTING:
        currentSuperState = SuperStates.SPINUP_SHOOTING;
        break;
      case AIMING:
        currentSuperState = SuperStates.AIMING;
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
      case SPINUP_SHOOTING:
        spinupShooting();
        break;
      case AIMING:
        aiming();
        break;
    }
  }

  private void aiming() {
    hood.setWantedState(HoodStates.AIMING);
  }

  private void spinupShooting() {
    // hood, flywheel
    hood.setWantedState(HoodStates.SPINUP_SHOOTING);
    flywheel.setWantedState(FlywheelStates.SPINUP_SHOOTING);
    if (hood.atSetpoint(8.0) && flywheel.atSetpoint(Constants.SPINUP_SHOOTING_FLYWHEEL_RPM, 100.0)) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SPINUP_SHOOTING);
      intake.setWantedState(IntakeStates.SPINUP_SHOOTING);
      indexer.setWantedState(IndexerStates.SPINUP_SHOOTING);
    }
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    indexer.setWantedState(Indexer.IndexerStates.INTAKING);
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

  public void stopSuperState(SuperStates state) {
    switch (state) {
      case INTAKING:
        if (intake.getState() == IntakeStates.INTAKING) intake.setWantedState(IntakeStates.OFF);
        if (indexer.getState() == IndexerStates.INTAKING) indexer.setWantedState(IndexerStates.OFF);
        break;
      case SPINUP_SHOOTING:
        if (intake.getState() == IntakeStates.SPINUP_SHOOTING)
          intake.setWantedState(IntakeStates.OFF);
        if (hood.getState() == HoodStates.SPINUP_SHOOTING) hood.setWantedState(HoodStates.OFF);
        if (indexer.getState() == IndexerStates.SPINUP_SHOOTING)
          indexer.setWantedState(IndexerStates.OFF);
        if (flywheel.getState() == FlywheelStates.SPINUP_SHOOTING)
          flywheel.setWantedState(FlywheelStates.OFF);
        if (flywheelKicker.getState() == FlywheelKickerStates.SPINUP_SHOOTING)
          flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
        break;
      default:
        stopped();
        break;
    }
  }

  public Command stopSuperStateCommand(SuperStates state) {
    return new InstantCommand(() -> stopSuperState(state), this);
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
