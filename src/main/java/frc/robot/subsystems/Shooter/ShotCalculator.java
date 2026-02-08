package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;

// how far we are from the hub
// taking that to convert to setpoints for flywhel and hood
public class ShotCalculator {
  private CommandSwerveDrivetrain drivetrain;

    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new
  InterpolatingDoubleTreeMap();

  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // distance from hub to hood angle
    // launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));

    // distance from hub to flywheel speed
    // launchFlywheelSpeedMap.put(1.34, 210.0);

    //idk what these values r rn
    //   timeOfFlightMap.put(5.68, 1.16);
  }

  // Calculate distance from turret to target
  private double getRobotDistanceFromHub() {
    Pose2d currentPosition = drivetrain.getPosition();
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double positionToTargetDistance = target.getDistance(currentPosition.getTranslation());
    return positionToTargetDistance;
  }

  private double getShooterDistanceFromHub(){
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double shooterToTargetDistance = target.getDistance(getShooterPosition().getTranslation());
    return shooterToTargetDistance;
  }

  private Pose2d getShooterPosition(){
    Pose2d shooterPosition =new Pose2d();
    shooterPosition = drivetrain.getPosition().transformBy(ShooterConstants.robotToShooter);
    return shooterPosition;
  }

  public Rotation2d getWantedHoodAngle() {
    return launchHoodAngleMap.get(getRobotDistanceFromHub());
  }

  private double getWantedFlywheelSpeed() {
    return launchFlywheelSpeedMap.get(getRobotDistanceFromHub());
  }

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
  // private LaunchingParameters latestParameters = null;

  // distance limits
  // private static double minDistance = 1.34;
  // private static double maxDistance= 5.60;
  // private static double phaseDelay;
  // cache check
  // public LaunchingParameters getParameters() {
  //   if (latestParameters != null) {
  //     return latestParameters;
  //   }

  // Calculate estimated pose while accounting for phase delay
  // Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
  // ChassisSpeeds robotRelativeVelocity = drivetrain.getVelocity();
  // estimatedPose =
  //     estimatedPose.exp(
  //         new Twist2d(
  //             robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
  //             robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
  //             robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

  // // Calculate field relative turret velocit

  // // Account for imparted velocity by robot (turret) to offset

//   private Pose2d getLookAheadPose(){    
//     Translation2d target =
//         AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
//     Pose2d drivetrainPos = drivetrain.getPosition();
//     Pose2d lookaheadPose = drivetrainPos;
//     double robotAngle = drivetrainPos.getRotation().getRadians();
//     ChassisSpeeds robotVelocity = drivetrain.getVelocity();
//     double velocityX =
//       robotVelocity.vxMetersPerSecond
//           + robotVelocity.omegaRadiansPerSecond
//               * (drivetrainPos.getY() * Math.cos(robotAngle)
//                   - drivetrainPos.getX() * Math.sin(robotAngle));
//   double velocityY =
//       robotVelocity.vyMetersPerSecond
//           + robotVelocity.omegaRadiansPerSecond
//               * (drivetrainPos.getX() * Math.cos(robotAngle)
//                   - drivetrainPos.getY() * Math.sin(robotAngle));
//     double timeOfFlight;
// double lookaheadTurretToTargetDistance = getRobotDistanceFromHub();
//   for (int i = 0; i < 20; i++) {
//     timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
//     double offsetX = velocityX * timeOfFlight;
//     double offsetY = velocityY * timeOfFlight;
//     lookaheadPose =
//         new Pose2d(
//             drivetrainPos.getTranslation().plus(new Translation2d(offsetX, offsetY)),
//             drivetrainPos.getRotation());
//     lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
//   }
//   return lookaheadPose;
//   }
  

  // Calculate parameters accounted for imparted velocity
  // turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
  // hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
  // if (lastTurretAngle == null) lastTurretAngle = turretAngle;
  // if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
  // turretVelocity =
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
  // Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
  // Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

  // return latestParameters;
  // }
// double shooterToTargetDistance = .getDistance(turretPosition.getTranslation());

//    // Calculate distance from turret to target
//     Translation2d target =
//         AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
//     Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
//     double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

//     // Calculate field relative turret velocity
//     ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
//     double robotAngle = estimatedPose.getRotation().getRadians();
//     double turretVelocityX =
//         robotVelocity.vxMetersPerSecond
//             + robotVelocity.omegaRadiansPerSecond
    //             * (robotToTurret.getY() * Math.cos(robotAngle)
    //                 - robotToTurret.getX() * Math.sin(robotAngle));
    // double turretVelocityY =
    //     robotVelocity.vyMetersPerSecond
    //         + robotVelocity.omegaRadiansPerSecond
    //             * (robotToTurret.getX() * Math.cos(robotAngle)
    //                 - robotToTurret.getY() * Math.sin(robotAngle));

    // // Account for imparted velocity by robot (turret) to offset
    // Pose2d  shooterPosition= drivetrain.getPosition().transformBy(ShooterConstants.robotToShooter);
    // double timeOfFlight;
    // Pose2d lookaheadPose = turretPosition;
    // double lookaheadTurretToTargetDistance = turretToTargetDistance;
    // for (int i = 0; i < 20; i++) {
    //   timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
    //   double offsetX = turretVelocityX * timeOfFlight;
    //   double offsetY = turretVelocityY * timeOfFlight;
    //   lookaheadPose =
    //       new Pose2d(
    //           turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
    //           turretPosition.getRotation());
    //   lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    // }

}
