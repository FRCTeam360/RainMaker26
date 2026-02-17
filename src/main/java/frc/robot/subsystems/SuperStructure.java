package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
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
  private CommandXboxController controller;

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
    AIMING
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
  private static final double FLYWHEEL_TOLERANCE_RPM = 500.0;
  private static final double KICKER_FEED_VELOCITY_RPM = 4500.0;
  private static final double INDEXER_FEED_DUTY_CYCLE = 0.4;
  private static final double STOPPED_VELOCITY_RPM = 0.0;

  public SuperStructure(
      Intake intake,
      Indexer indexer,
      FlywheelKicker flywheelKicker,
      Flywheel flywheel,
      Hood hood,
      CommandSwerveDrivetrain driveTrain,
      ShotCalculator shotCalculator,
      CommandXboxController controller) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.drivetrain = driveTrain;
    this.shotCalculator = shotCalculator;
    this.controller = controller;
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
        fieldOrientedDrive();
        intaking();
        break;
      case IDLE:
        fieldOrientedDrive();
        stopped();
        break;
      case SHOOTING:
        shooting();
        updateShooterStates();
        applyShooterStates();
        break;
      case AIMING:
        fieldOrientedDrive();
        aiming();
        break;
    }
  }

  private void updateShooterStates() {
    previousShooterState = currentShooterState;
    if (flywheel.atSetpoint(shotCalculator.calculateShot().flywheelSpeed(), FLYWHEEL_TOLERANCE_RPM)
        && hood.atSetpoint(shotCalculator.calculateShot().hoodAngle())) {
      currentShooterState = ShooterStates.FIRING;
    } else {
      currentShooterState = ShooterStates.PREPARING;
    }
  }

  private void applyShooterStates() {
    if (currentShooterState == ShooterStates.FIRING) {
      flywheelKicker.setVelocity(KICKER_FEED_VELOCITY_RPM);
      indexer.setDutyCycle(INDEXER_FEED_DUTY_CYCLE);
    } else {
      flywheelKicker.setVelocity(STOPPED_VELOCITY_RPM);
    }
  }

  private void aiming() {
    hood.setWantedState(HoodStates.AIMING);
  }

  private void shooting() {
    flywheel.setVelocity(shotCalculator.calculateShot().flywheelSpeed());
    hood.setPosition(shotCalculator.calculateShot().hoodAngle());
    drivetrain.faceAngleWhileDriving(controller, shotCalculator.calculateShot().targetHeading());
  }

  private void fieldOrientedDrive() {
    if (DriverStation.isTeleopEnabled()) {
      drivetrain.fieldOrientedDrive(controller);
    }
  }

  private void intaking() {
    intake.setWantedState(Intake.IntakeStates.INTAKING);
    // indexer.setWantedState(Indexer.IndexerStates.INTAKING);
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
    Logger.recordOutput("Superstructure/PreviousShooterState", previousShooterState.toString());
    Logger.recordOutput("Superstructure/CurrentShooterState", currentShooterState.toString());
  }
}
