package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.CommandLogger;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem using the AdvantageKit IO-layer architecture (6328 template). Replaces the
 * previous CTRE CommandSwerveDrivetrain approach with explicit hardware abstraction for full replay
 * support.
 *
 * <p>NOTE: The drive PID gains (both module-level and path-following) need to be retuned after this
 * migration. The previous CTRE implementation ran the heading controller at 250Hz on the odometry
 * thread; this implementation runs all control at the 50Hz scheduler rate. Gains that worked at
 * 250Hz will be too aggressive at 50Hz.
 */
public class Drive extends SubsystemBase {
  // Odometry frequency based on CAN bus type
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  // Drive base radius computed from module positions
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // Max speeds
  public final LinearVelocity maxSpeed = Constants.getMaxSpeed();
  private final double maxSpeedMps = maxSpeed.in(MetersPerSecond);
  private final double maxAngularSpeedRadPerSec = maxSpeedMps / DRIVE_BASE_RADIUS;

  // PathPlanner config
  private static final double ROBOT_MASS_KG = 60.0;
  private static final double ROBOT_MOI = 4.5;
  private static final double WHEEL_COF = 1.3;

  // TODO: RETUNE - PathPlanner PID gains (previously tuned for CTRE's 250Hz loop)
  private static final double PP_TRANSLATION_KP = 5.0;
  private static final double PP_TRANSLATION_KI = 0.0;
  private static final double PP_TRANSLATION_KD = 0.0;
  private static final double PP_ROTATION_KP = 5.0;
  private static final double PP_ROTATION_KI = 0.0;
  private static final double PP_ROTATION_KD = 0.0;

  // TODO: RETUNE - Heading controller PID gains (now runs at 50Hz instead of 250Hz)
  private static final double HEADING_KP = 6.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.0;
  private static final double HEADING_TOLERANCE_RAD = Math.toRadians(5.0);
  private static final double HEADING_SPEED_TOLERANCE_RAD_PER_MPS = Math.toRadians(1.0);

  // TODO: RETUNE - Auto rotation override PID
  private static final double PP_OVERRIDE_KP = 5.0;
  private static final double PP_OVERRIDE_KI = 0.0;
  private static final double PP_OVERRIDE_KD = 0.0;

  // TODO: RETUNE - BLine PID gains
  private static final double BLINE_TRANSLATION_KP = 5.0;
  private static final double BLINE_TRANSLATION_KI = 0.0;
  private static final double BLINE_TRANSLATION_KD = 0.0;
  private static final double BLINE_ROTATION_KP = 3.0;
  private static final double BLINE_ROTATION_KI = 0.0;
  private static final double BLINE_ROTATION_KD = 0.0;
  private static final double BLINE_CROSS_TRACK_KP = 2.5;
  private static final double BLINE_CROSS_TRACK_KI = 0.0;
  private static final double BLINE_CROSS_TRACK_KD = 0.0;

  // Driver control constants
  private static final double CONTROLLER_DEADBAND = 0.04;
  private static final double FACING_ANGLE_MAX_SPEED_FRACTION = 0.5;
  private static final double SNAP_THRESHOLD = 0.3;

  private static final String LOG_PREFIX = "Swerve/";

  // Odometry lock shared with PhoenixOdometryThread
  static final Lock odometryLock = new ReentrantLock();

  // Hardware IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Kinematics and odometry
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  // Heading controller for face-angle commands
  private final PIDController headingController =
      new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

  // PathPlanner rotation override controller
  private final PIDController ppRotationOverrideController =
      new PIDController(PP_OVERRIDE_KP, PP_OVERRIDE_KI, PP_OVERRIDE_KD);
  private boolean autoRotationActive = false;

  // Commanded speeds for shoot-on-the-move compensation
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();

  // Defense mode
  private boolean isDefenseMode = false;

  // Heading lock state
  private boolean headingLockEnabled = false;
  private double currentTargetAngle = 0.0;

  private enum SnapDirection {
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT
  }

  private SnapDirection previousSnapDirection = SnapDirection.NONE;

  // SysId
  private final SysIdRoutine sysId;

  // BLine logging key cache
  private final Map<String, String> blineKeyCache = new HashMap<>();

  // Vision tracking
  private boolean hasVisionMeasurements = false;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Configure heading controllers
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(HEADING_TOLERANCE_RAD);

    ppRotationOverrideController.enableContinuousInput(-Math.PI, Math.PI);
    ppRotationOverrideController.setTolerance(HEADING_TOLERANCE_RAD);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure PathPlanner AutoBuilder
    configureAutoBuilder();
    configureAutoLogging();

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  // ========================= PERIODIC =========================

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(
        !gyroInputs.connected && Constants.getRobotType() != Constants.RobotType.SIM);

    // Logging
    Logger.recordOutput(LOG_PREFIX + "CurrentPose", getPose());
    Logger.recordOutput(LOG_PREFIX + "Rotation2d", getRotation());
    Logger.recordOutput(LOG_PREFIX + "Using Vision", hasVisionMeasurements);
    Logger.recordOutput(LOG_PREFIX + "Is Defense Mode", isDefenseMode);
    Logger.recordOutput(LOG_PREFIX + "HeadingLockEnabled", headingLockEnabled);
  }

  // ========================= DRIVE CONTROL =========================

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Drives the robot with the given robot-relative chassis speeds. */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    commandedSpeeds = speeds;
    runVelocity(speeds);
  }

  /** Runs the drive in a straight line with the specified drive output (for characterization). */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Stops the drive and turns modules to X arrangement to resist movement. */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // ========================= TELEOP COMMANDS =========================

  /** Main field-oriented drive command with cubic response curve and heading lock support. */
  public Command fieldOrientedDriveCommand(CommandXboxController driveCont) {
    double defenseModeRotationScaler = 1.25;
    double defenseModeTranslationScaler = 0.75;

    return this.run(
        () -> {
          double velXMps = Math.pow(driveCont.getLeftY(), 3) * maxSpeedMps * -1.0;
          double velYMps = Math.pow(driveCont.getLeftX(), 3) * maxSpeedMps * -1.0;
          double omegaRps =
              Math.pow(driveCont.getRightX(), 2)
                  * maxAngularSpeedRadPerSec
                  * -Math.signum(driveCont.getRightX());

          if (isDefenseMode) {
            velXMps *= defenseModeTranslationScaler;
            velYMps *= defenseModeTranslationScaler;
            omegaRps *= defenseModeRotationScaler;
          }

          boolean isBlueAlliance =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          double fieldVelX = isBlueAlliance ? velXMps : -velXMps;
          double fieldVelY = isBlueAlliance ? velYMps : -velYMps;

          if (headingLockEnabled) {
            updateSnapAngle(driveCont.getRightX(), driveCont.getRightY());
            double headingOutput =
                headingController.calculate(
                    getRotation().getRadians(), Math.toRadians(currentTargetAngle));
            ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldVelX, fieldVelY, headingOutput, getRotation());
            commandedSpeeds = speeds;
            runVelocity(speeds);
          } else {
            ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldVelX, fieldVelY, omegaRps, getRotation());
            commandedSpeeds = speeds;
            runVelocity(speeds);
          }
        });
  }

  /** Drives field-centric while continuously rotating to face the given heading. */
  public void faceAngleWhileDriving(
      double fieldRelativeVelXMps, double fieldRelativeVelYMps, Rotation2d targetHeading) {
    double speedCapMps = maxSpeedMps * FACING_ANGLE_MAX_SPEED_FRACTION;

    double rawSpeedMps = Math.hypot(fieldRelativeVelXMps, fieldRelativeVelYMps);
    if (rawSpeedMps > speedCapMps) {
      double scale = speedCapMps / rawSpeedMps;
      fieldRelativeVelXMps *= scale;
      fieldRelativeVelYMps *= scale;
    }

    double omegaRps =
        headingController.calculate(getRotation().getRadians(), targetHeading.getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeVelXMps, fieldRelativeVelYMps, omegaRps, getRotation());
    commandedSpeeds = speeds;
    runVelocity(speeds);
  }

  /** Creates a command that drives while facing a target heading. */
  public Command faceAngleWhileDrivingCommand(
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      Supplier<Rotation2d> headingSupplier) {
    return new FunctionalCommand(
        () -> headingController.reset(),
        () -> {
          double rawVelXMps = velocityXSupplier.getAsDouble();
          double rawVelYMps = velocityYSupplier.getAsDouble();

          boolean isBlueAlliance =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          double fieldVelXMps = isBlueAlliance ? rawVelXMps : -rawVelXMps;
          double fieldVelYMps = isBlueAlliance ? rawVelYMps : -rawVelYMps;

          faceAngleWhileDriving(fieldVelXMps, fieldVelYMps, headingSupplier.get());
        },
        interrupted -> headingController.reset(),
        () -> false,
        this);
  }

  /** Creates a face-angle command using controller input with cubic response curve. */
  public Command faceAngleWhileDrivingCommand(
      CommandXboxController driveCont, Supplier<Rotation2d> headingSupplier) {
    return faceAngleWhileDrivingCommand(
        () -> Math.pow(driveCont.getLeftY(), 3) * maxSpeedMps * -1.0,
        () -> Math.pow(driveCont.getLeftX(), 3) * maxSpeedMps * -1.0,
        headingSupplier);
  }

  /** X-out: locks wheels in an X pattern to resist movement. */
  public void xOut() {
    commandedSpeeds = new ChassisSpeeds();
    stopWithX();
  }

  public Command xOutCmd() {
    return CommandLogger.logCommand(this.runEnd(() -> this.xOut(), () -> this.xOut()), "X_OUT");
  }

  // ========================= HEADING LOCK =========================

  public Command toggleHeadingLockCommand() {
    return new InstantCommand(
        () -> {
          headingLockEnabled = !headingLockEnabled;
          if (headingLockEnabled) {
            headingController.reset();
            double currentDegrees = getRotation().getDegrees();
            currentDegrees = ((currentDegrees % 360) + 360) % 360;
            currentTargetAngle = Math.round(currentDegrees / 90.0) * 90.0;
            if (currentTargetAngle >= 360) currentTargetAngle = 0;
          }
        });
  }

  public boolean isHeadingLockEnabled() {
    return headingLockEnabled;
  }

  private void updateSnapAngle(double rightX, double rightY) {
    boolean yDominant = Math.abs(rightY) >= Math.abs(rightX);
    boolean pushUp = yDominant && rightY < -SNAP_THRESHOLD;
    boolean pushDown = yDominant && rightY > SNAP_THRESHOLD;
    boolean pushRight = !yDominant && rightX > SNAP_THRESHOLD;
    boolean pushLeft = !yDominant && rightX < -SNAP_THRESHOLD;

    SnapDirection currentSnapDirection;
    if (pushUp) currentSnapDirection = SnapDirection.UP;
    else if (pushDown) currentSnapDirection = SnapDirection.DOWN;
    else if (pushRight) currentSnapDirection = SnapDirection.RIGHT;
    else if (pushLeft) currentSnapDirection = SnapDirection.LEFT;
    else currentSnapDirection = SnapDirection.NONE;

    if (currentSnapDirection == previousSnapDirection) return;
    previousSnapDirection = currentSnapDirection;

    switch (currentSnapDirection) {
      case UP ->
          currentTargetAngle = AllianceFlipUtil.apply(Rotation2d.fromDegrees(0.0)).getDegrees();
      case DOWN ->
          currentTargetAngle = AllianceFlipUtil.apply(Rotation2d.fromDegrees(180.0)).getDegrees();
      case RIGHT ->
          currentTargetAngle = AllianceFlipUtil.apply(Rotation2d.fromDegrees(270.0)).getDegrees();
      case LEFT ->
          currentTargetAngle = AllianceFlipUtil.apply(Rotation2d.fromDegrees(90.0)).getDegrees();
      case NONE -> {}
    }
  }

  // ========================= DEFENSE MODE =========================

  public Command toggleDefenseModeCmd() {
    return this.runOnce(() -> isDefenseMode = !isDefenseMode);
  }

  // ========================= HEADING ALIGNMENT =========================

  /**
   * Returns whether the heading controller is aligned to its target within a speed-scaled
   * tolerance.
   */
  public boolean isAlignedToTarget() {
    if (autoRotationActive) {
      return isAlignedToAutoTarget();
    }

    ChassisSpeeds speeds = getChassisSpeeds();
    double speedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double dynamicToleranceRad =
        HEADING_TOLERANCE_RAD + speedMps * HEADING_SPEED_TOLERANCE_RAD_PER_MPS;
    Logger.recordOutput(
        LOG_PREFIX + "DynamicHeadingToleranceDeg", Math.toDegrees(dynamicToleranceRad));

    headingController.setTolerance(dynamicToleranceRad);
    return headingController.atSetpoint();
  }

  /** Resets the heading controller. */
  public void resetHeadingController() {
    headingController.reset();
  }

  // ========================= AUTO ROTATION OVERRIDE =========================

  /**
   * Sets the rotation override target for PathPlanner path-following.
   *
   * @param headingSupplier supplies the target heading each cycle
   */
  public void setAutoRotationOverride(Supplier<Rotation2d> headingSupplier) {
    ppRotationOverrideController.reset();
    autoRotationActive = true;
    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Rotation2d target = headingSupplier.get();
          double output =
              ppRotationOverrideController.calculate(
                  getRotation().getRadians(), target.getRadians());
          Logger.recordOutput(LOG_PREFIX + "AutoRotationOverride/TargetDeg", target.getDegrees());
          Logger.recordOutput(
              LOG_PREFIX + "AutoRotationOverride/ErrorDeg",
              Math.toDegrees(ppRotationOverrideController.getPositionError()));
          Logger.recordOutput(LOG_PREFIX + "AutoRotationOverride/OutputRadPerS", output);
          return output;
        });
    Logger.recordOutput(LOG_PREFIX + "AutoRotationOverride/Active", true);
  }

  /** Clears the rotation override, returning control to PathPlanner's built-in controller. */
  public void clearAutoRotationOverride() {
    autoRotationActive = false;
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    ppRotationOverrideController.reset();
    Logger.recordOutput(LOG_PREFIX + "AutoRotationOverride/Active", false);
  }

  /** Returns whether the auto rotation override PID is at its setpoint. */
  public boolean isAlignedToAutoTarget() {
    if (!autoRotationActive) return false;
    ChassisSpeeds speeds = getChassisSpeeds();
    double speedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double dynamicToleranceRad =
        HEADING_TOLERANCE_RAD + speedMps * HEADING_SPEED_TOLERANCE_RAD_PER_MPS;
    ppRotationOverrideController.setTolerance(dynamicToleranceRad);
    return ppRotationOverrideController.atSetpoint();
  }

  // ========================= PATHPLANNER =========================

  private void configureAutoBuilder() {
    RobotConfig config =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            getModuleTranslations());

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> {
          commandedSpeeds = speeds;
          runVelocity(speeds);
        },
        new PPHolonomicDriveController(
            new PIDConstants(PP_TRANSLATION_KP, PP_TRANSLATION_KI, PP_TRANSLATION_KD),
            new PIDConstants(PP_ROTATION_KP, PP_ROTATION_KI, PP_ROTATION_KD)),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  private void configureAutoLogging() {
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("Autos/PathPlanner/currentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Autos/PathPlanner/targetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("Autos/PathPlanner/activePath", path.toArray(new Pose2d[0])));

    FollowPath.setPoseLoggingConsumer(
        (Pair<String, Pose2d> pair) ->
            Logger.recordOutput(blineKey(pair.getFirst()), pair.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        (Pair<String, Translation2d[]> pair) ->
            Logger.recordOutput(blineKey(pair.getFirst()), pair.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        (Pair<String, Boolean> pair) ->
            Logger.recordOutput(blineKey(pair.getFirst()), pair.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        (Pair<String, Double> pair) ->
            Logger.recordOutput(blineKey(pair.getFirst()), pair.getSecond()));
  }

  private String blineKey(String rawKey) {
    return blineKeyCache.computeIfAbsent(
        rawKey, k -> "Autos/BLine/" + k.replace("FollowPath/", ""));
  }

  // ========================= BLINE =========================

  /**
   * Creates a reusable BLine FollowPath.Builder configured for this drivetrain.
   *
   * @return a configured {@link FollowPath.Builder}
   */
  public FollowPath.Builder createBLinePathBuilder() {
    return new FollowPath.Builder(
            this,
            this::getPose,
            this::getChassisSpeeds,
            this::driveRobotRelative,
            new PIDController(BLINE_TRANSLATION_KP, BLINE_TRANSLATION_KI, BLINE_TRANSLATION_KD),
            new PIDController(BLINE_ROTATION_KP, BLINE_ROTATION_KI, BLINE_ROTATION_KD),
            new PIDController(BLINE_CROSS_TRACK_KP, BLINE_CROSS_TRACK_KI, BLINE_CROSS_TRACK_KD))
        .withPoseReset(this::resetPose);
  }

  // ========================= SYSID =========================

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  // ========================= VISION =========================

  public void addVisionMeasurements(List<VisionMeasurement> measurements) {
    hasVisionMeasurements = !measurements.isEmpty();
    for (VisionMeasurement measurement : measurements) {
      poseEstimator.addVisionMeasurement(
          measurement.estimatedPose(), measurement.timestamp(), measurement.standardDeviation());
    }
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // ========================= POSE & STATE GETTERS =========================

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Alias for getPose() - matches CommandSwerveDrivetrain API. */
  public Pose2d getPosition() {
    return getPose();
  }

  public Pose2d getPose2d() {
    return getPose();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Rotation2d getRotation2d() {
    return getRotation();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the robot's heading in degrees. */
  public double getAngle() {
    return getRotation().getDegrees();
  }

  /** Returns the robot's angular velocity in degrees/second. */
  public double getAngularRate() {
    return Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond);
  }

  /** Resets the gyro heading. */
  public void zero() {
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
  }

  public Command zeroCommand() {
    return this.runOnce(() -> zero());
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Alias for getChassisSpeeds() - matches old API. */
  public ChassisSpeeds getVelocity() {
    return getChassisSpeeds();
  }

  /** Returns the commanded chassis speeds (less noisy for shoot-on-the-move). */
  public ChassisSpeeds getCommandedVelocity() {
    return commandedSpeeds;
  }

  /** Returns the heading tolerance in radians. */
  public double getHeadingToleranceRad() {
    return HEADING_TOLERANCE_RAD;
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMps;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeedRadPerSec;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
