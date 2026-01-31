package frc.robot.subsystems.Shooter;

import java.lang.System.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


//@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
    private CommandSwerveDrivetrain drivetrain;
    private static ShotCalculator instance;

    private final LinearFilter turretAngleFilter = 
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
    private final LinearFilter hoodAngleFilter = 
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;
    private Rotation2d turretAngle;
    private double hoodAngle = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;

    public static ShotCalculator getInstance(CommandSwerveDrivetrain drivetrain) {
        if (instance == null) instance = new ShotCalculator(drivetrain);
        return instance;
    }
    public ShotCalculator(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    public record ShootingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed) {}

    private ShootingParameters latestParameters = null;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOflightMap = 
        new InterpolatingDoubleTreeMap();

    static {
        minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;

    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) {
            return latestParameters;
        }
    }

    Pose2d estimatedPose = drivetrain.getPosition();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
    
    Translation2d target = 
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
    double shooterToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // calc field relative turret velocity, not doing

    // Account or imparted velocity by robot (turret) to offset, not doing

    // Calculate parameters accounted for imparted velocity, not doing

    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose); // i think something is incomplete somewhere else.
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

    return latestParameters;
}

public void clearShootingParameters() {
    latestParameters = null;
}
