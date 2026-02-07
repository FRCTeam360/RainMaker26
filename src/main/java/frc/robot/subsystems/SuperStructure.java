package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker.FlywheelKickerStates;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.AllianceFlipUtil;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodStates;
import frc.robot.utils.FieldConstants;

import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final FlywheelKicker flywheelKicker;
  private Flywheel flywheel;
  private Hood hood;
  private ShotCalculator shotCalculator;

  public enum SuperStates {
    IDLE, // everything is stopped when nothing else happens
    DEFENSE, // driver holds defense button -> less desired velocitu moving latterally, more into
    // rotation in drivetrain
    X_OUT, // hold down button to x out wheels or press once and wheels stop X-ing out when moved
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
      Intake intake, Indexer indexer, FlywheelKicker flywheelKicker, Flywheel flywheel, Hood hood, ShotCalculator shotCalculator) {
    this.intake = intake;
    this.indexer = indexer;
    this.flywheelKicker = flywheelKicker;
    this.flywheel = flywheel;
    this.hood = hood;
    this.shotCalculator = shotCalculator;
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
      case AIMING:
      currentSuperState = SuperStates.AIMING;
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
      case AIMING:
        aiming();
        break;
    }
  }

  private void aiming(){
    
  }


  private void spinupShooting() {
    // hood, flywheel
    hood.setWantedState(HoodStates.SPINUP_SHOOTING);
    flywheel.setWantedState(FlywheelStates.SPINUP_SHOOTING);
    if (hood.atSetpoint(6.0) && flywheel.atSetpoint(3000.0, 100.0)) {
      flywheelKicker.setWantedState(FlywheelKickerStates.SPINUP_SHOOTING);
      intake.setWantedState(IntakeStates.INTAKING);
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

  //how far we are from the hub
  //taking that to convert to setpoints for flywhel and hood
  class ShotCalculator{
    private CommandSwerveDrivetrain drivetrain;

    public ShotCalculator(CommandSwerveDrivetrain drivetrain){
      this.drivetrain = drivetrain;
    }
    Pose2d currentPosition = drivetrain.getPosition();

    // Calculate distance from turret to target
    Translation2d target =
    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double positionToTargetDistance = target.getDistance(currentPosition.getTranslation());

    // // moving average filter arranged ove r~.1 sec,
    // // smooths angular velocityestimates
    // // without this u get sensor noise and violent velocity spikes
    // private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / .02));
    // private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / .02));

    // private Rotation2d lastTurretAngle;
    // private double lastHoodAngle;

    // outputs
    // private Rotation2d turretAngle;
    // private double hoodAngle = Double.NaN;
    // private double turretVelocity;
    // private double hoodVelocity;

    // isvalid checks if we can shoots
    // turret angle where to aim
    // turret velocity feedforward term
    // hood angle is elevation
    // hood velocity is feedforward
    // flywheelspeed is RPM

    // public record LaunchingParameters(
    //         boolean isValid,
    //         Rotation2d turretAngle,
    //         double turretVelocity,
    //         double hoodAngle,
    //         double hoodVelocity,
    //         double flywheelSpeed) {
    // }

    // Cache parameters
    //private LaunchingParameters latestParameters = null;

    // distance limits
    private static double minDistance = 1.34;
    private static double maxDistance= 5.60;
    //private static double phaseDelay;

    // private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap = new InterpolatingTreeMap<>(
    //         InverseInterpolator.forDouble(), Rotation2d::interpolate);
    // private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();


    //launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    // launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    // launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    // launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    // launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    // launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    // launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    // launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    // launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    // launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    //launchFlywheelSpeedMap.put(1.34, 210.0);
    // launchFlywheelSpeedMap.put(1.78, 220.0);
    // launchFlywheelSpeedMap.put(2.17, 220.0);
    // launchFlywheelSpeedMap.put(2.81, 230.0);
    // launchFlywheelSpeedMap.put(3.82, 250.0);
    // launchFlywheelSpeedMap.put(4.09, 255.0);
    // launchFlywheelSpeedMap.put(4.40, 260.0);
    // launchFlywheelSpeedMap.put(4.77, 265.0);
    // launchFlywheelSpeedMap.put(5.57, 275.0);
    // launchFlywheelSpeedMap.put(5.60, 290.0);

   //timeOfFlightMap.put(5.68, 1.16);
    // timeOfFlightMap.put(4.55, 1.12);
    // timeOfFlightMap.put(3.15, 1.11);
    // timeOfFlightMap.put(1.88, 1.09);
    // timeOfFlightMap.put(1.38, 0.90);

  //cache check
  // public LaunchingParameters getParameters() {
  //   if (latestParameters != null) {
  //     return latestParameters;
  //   }

    // Calculate estimated pose while accounting for phase delay
    // Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));


    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drivetrain.getVelocity();
    double robotAngle = currentPosition.getRotation().getRadians();
    double velocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (currentPosition.getY() * Math.cos(robotAngle)
                    - currentPosition.getX() * Math.sin(robotAngle));
    double velocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (currentPosition.getX() * Math.cos(robotAngle)
                    - currentPosition.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = currentPosition;
    double lookaheadTurretToTargetDistance = positionToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = velocityX * timeOfFlight;
      double offsetY = velocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              currentPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              currentPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    //turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
   // hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    //if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    //if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    //turretVelocity =
        // turretAngleFilter.calculate(
        //     turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    // hoodVelocity =
    //     hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    // lastTurretAngle = turretAngle;
    // lastHoodAngle = hoodAngle;
    // latestParameters =
    //     new LaunchingParameters(
    //         lookaheadTurretToTargetDistance >= minDistance
    //             && lookaheadTurretToTargetDistance <= maxDistance,
    //         turretAngle,
    //         turretVelocity,
    //         hoodAngle,
    //         hoodVelocity,
    //         launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }
}
