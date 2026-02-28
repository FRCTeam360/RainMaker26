// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.PracticeBotDrivetrain;
import frc.robot.generated.WoodBotDrivetrain;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIONoop;
import frc.robot.subsystems.Climber.ClimberIOPB;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ControlState;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.HopperRoller.HopperRollerIONoop;
import frc.robot.subsystems.HopperRoller.HopperRollerIOPB;
import frc.robot.subsystems.HopperRoller.HopperRollerIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOPB;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOPB;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivotIONoop;
import frc.robot.subsystems.IntakePivot.IntakePivotIOPB;
import frc.robot.subsystems.IntakePivot.IntakePivotIOSim;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOPB;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOWBBangBang;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKickerIOPB;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKickerIOSim;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKickerIOWB;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.HoodIOPB;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOWB;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.RobotShootingInfo;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperInternalStates;
import frc.robot.subsystems.SuperStructure.SuperWantedStates;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight3G;
import frc.robot.subsystems.Vision.VisionIOLimelight4;
import frc.robot.subsystems.Vision.VisionIOLimelightBase;
import frc.robot.subsystems.Vision.VisionIOPhotonSim;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.PositionUtils;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private CommandSwerveDrivetrain drivetrain;
  private SendableChooser<Command> autoChooser;
  private Flywheel flywheel;
  private Hood hood;
  private Indexer indexer;
  private Vision vision;
  private Intake intake;
  private IntakePivot intakePivot;
  private HopperRoller hopperRoller;
  private FlywheelKicker flywheelKicker;
  private Climber climber;
  private BooleanSupplier canShootInHub;

  private SuperStructure superStructure;

  private ShotCalculator hubShotCalculator;
  private ShotCalculator passCalculator;

  // TODO: refactor to allow for more than 1 drivetrain type

  private Telemetry logger;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverCont = new CommandXboxController(0);
  private final CommandXboxController operatorCont = new CommandXboxController(1);
  private final CommandXboxController testCont1 = new CommandXboxController(5);

  private static final double FLYWHEEL_KICKER_WARMUP_VELOCITY_RPM = 4000.0;

  /** Frames to skip between processed frames while disabled. Only affects Limelight 4. */
  private static final int DISABLED_THROTTLE_SKIP_FRAMES = 200;

  /** Frames to skip between processed frames while enabled. Only affects Limelight 4. */
  private static final int ENABLED_THROTTLE_SKIP_FRAMES = 0;

  private RobotShootingInfo robotShootingInfo;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobotType()) {
      case SIM:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        climber = new Climber(new ClimberIOSim());
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        vision =
            new Vision(
                Map.of("photonSim", new VisionIOPhotonSim(() -> drivetrain.getState().Pose)));
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOSim());
        hopperRoller = new HopperRoller(new HopperRollerIOSim());

        robotShootingInfo =
            new RobotShootingInfo(
                Constants.WoodBotConstants.shotHoodAngleMap,
                Constants.WoodBotConstants.shotFlywheelSpeedMap,
                Constants.WoodBotConstants.timeOfFlightMap,
                ShooterConstants.WOODBOT_TO_SHOOTER,
                Constants.WoodBotConstants.MIN_SHOT_DISTANCE_METERS,
                Constants.WoodBotConstants.MAX_SHOT_DISTANCE_METERS);
        break;
      case WOODBOT:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        climber = new Climber(new ClimberIONoop());
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOWBBangBang());
        hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        vision =
            new Vision(
                Map.ofEntries(
                    Map.entry(
                        Constants.WoodBotConstants.LIMELIGHT_3,
                        new VisionIOLimelight3G(
                            Constants.WoodBotConstants.LIMELIGHT_3,
                            () -> drivetrain.getAngle(),
                            () -> drivetrain.getAngularRate(),
                            true)),
                    Map.entry(
                        Constants.WoodBotConstants.LIMELIGHT_4,
                        new VisionIOLimelight4(
                            Constants.WoodBotConstants.LIMELIGHT_4,
                            () -> drivetrain.getAngle(),
                            () -> drivetrain.getAngularRate(),
                            false))));
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        intakePivot = new IntakePivot(new IntakePivotIONoop());
        hopperRoller = new HopperRoller(new HopperRollerIONoop());

        robotShootingInfo =
            new RobotShootingInfo(
                Constants.WoodBotConstants.shotHoodAngleMap,
                Constants.WoodBotConstants.shotFlywheelSpeedMap,
                Constants.WoodBotConstants.timeOfFlightMap,
                ShooterConstants.WOODBOT_TO_SHOOTER,
                Constants.WoodBotConstants.MIN_SHOT_DISTANCE_METERS,
                Constants.WoodBotConstants.MAX_SHOT_DISTANCE_METERS);
        break;
      case PRACTICEBOT:
      default:
        drivetrain = PracticeBotDrivetrain.createDrivetrain();
        climber = new Climber(new ClimberIOPB());
        logger = new Telemetry(PracticeBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOPB());
        hood = new Hood(new HoodIOPB());
        indexer = new Indexer(new IndexerIOPB());
        vision = // TODO ADD OTHER LIMELIGHTS
            new Vision(
                Map.ofEntries(
                    Map.entry(
                        Constants.PracticeBotConstants.LIMELIGHT,
                        new VisionIOLimelight4(
                            Constants.PracticeBotConstants.LIMELIGHT,
                            () -> drivetrain.getAngle(),
                            () -> drivetrain.getAngularRate(),
                            true))));
        intake = new Intake(new IntakeIOPB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOPB());
        intakePivot = new IntakePivot(new IntakePivotIOPB());
        hopperRoller = new HopperRoller(new HopperRollerIOPB());

        robotShootingInfo =
            new RobotShootingInfo(
                Constants.PracticeBotConstants.shotHoodAngleMap,
                Constants.PracticeBotConstants.shotFlywheelSpeedMap,
                Constants.PracticeBotConstants.timeOfFlightMap,
                ShooterConstants.PRACTICEBOT_TO_SHOOTER,
                Constants.PracticeBotConstants.MIN_SHOT_DISTANCE_METERS,
                Constants.PracticeBotConstants.MAX_SHOT_DISTANCE_METERS);
        // TODO ADD CLIMBERS
        break;
    }
    hubShotCalculator =
        new ShotCalculator(
            drivetrain::getPosition,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
            robotShootingInfo);
    passCalculator =
        new ShotCalculator(
            drivetrain::getPosition,
            () ->
                PositionUtils.getCloserPassTarget(
                    drivetrain.getPosition(),
                    AllianceFlipUtil.apply(FieldConstants.RightBump.nearRightCorner),
                    AllianceFlipUtil.apply(FieldConstants.LeftBump.nearLeftCorner)),
            robotShootingInfo);
    // Configure the trigger bindings
    // TODO: Re-enable superStructure construction and PathPlanner commands

    superStructure =
        new SuperStructure(
            intake,
            indexer,
            flywheelKicker,
            flywheel,
            hood,
            intakePivot,
            hopperRoller,
            hubShotCalculator,
            passCalculator,
            drivetrain::isAlignedToTarget,
            drivetrain::getPosition,
            robotShootingInfo.robotToShooter());
    intake.setDutyCycleSupplier(driverCont::getLeftTriggerAxis);

    registerPathplannerCommand(
        "basic intake", superStructure.setStateCommand(SuperWantedStates.INTAKING));
    // TODO: add end condition based on state from SuperStructure (based on sensor inputs)
    registerPathplannerCommand(
        "shoot at hub",
        Commands.waitSeconds(10)
            .deadlineFor(
                superStructure
                    .setStateCommand(SuperWantedStates.SHOOT_AT_HUB)
                    .alongWith(
                        drivetrain.faceAngleWhileDrivingCommand(
                            () -> 0,
                            () -> 0,
                            () -> hubShotCalculator.calculateShot().targetHeading())))
            .andThen(superStructure.setStateCommand(SuperWantedStates.DEFAULT)));

    configVision();
    configDefaultDrivingCommand();
    configureBindings();
    // configureTestBindings();
    // configureFullShootingTestBindings();
    // configureFullShootingTestBindings();

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  public void registerPathplannerCommand(String name, Command command) {
    if (Objects.nonNull(command)) {
      NamedCommands.registerCommand(name, command);
    } else {
      System.err.println(name + " is null");
      NamedCommands.registerCommand(
          name, new InstantCommand(() -> System.err.println(name + " is null")));
    }
  }

  private void configVision() {
    Command consumeVisionMeasurements =
        vision.consumeVisionMeasurements(
            measurements -> {
              drivetrain.addVisionMeasurements(measurements);
            });
    vision.setDefaultCommand(consumeVisionMeasurements.ignoringDisable(true));
  }

  private void configDefaultDrivingCommand() {
    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDriveCommand(driverCont));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    BooleanSupplier isSuperstructureMode =
        () -> superStructure.getControlState() == ControlState.SUPERSTRUCTURE;
    BooleanSupplier isIndependentMode =
        () -> superStructure.getControlState() == ControlState.INDEPENDENT;

    // Auto-cycle: right trigger auto-selects hub or outpost based on alliance zone position.
    // The heading supplier dynamically picks the correct calculator based on resolved state.
    Trigger autoCycleTrigger = driverCont.rightTrigger().and(isSuperstructureMode);
    autoCycleTrigger.whileTrue(
        superStructure
            .setStateCommand(SuperWantedStates.AUTO_CYCLE_SHOOTING)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    driverCont,
                    () -> {
                      if (superStructure.getCurrentSuperState() == SuperInternalStates.PASSING) {
                        return passCalculator.calculateShot().targetHeading();
                      }
                      return hubShotCalculator.calculateShot().targetHeading();
                    })));
    autoCycleTrigger.onFalse(superStructure.setStateCommand(SuperWantedStates.DEFAULT));

    // Manual override: force shoot at hub regardless of position
    Trigger forceHubTrigger = driverCont.rightBumper().and(isSuperstructureMode);
    forceHubTrigger.whileTrue(
        superStructure
            .setStateCommand(SuperWantedStates.SHOOT_AT_HUB)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    driverCont, () -> hubShotCalculator.calculateShot().targetHeading())));
    forceHubTrigger.onFalse(superStructure.setStateCommand(SuperWantedStates.DEFAULT));

    // Manual override: force pass to outpost regardless of position
    Trigger forceOutpostTrigger = driverCont.leftBumper().and(isSuperstructureMode);
    forceOutpostTrigger.whileTrue(
        superStructure
            .setStateCommand(SuperWantedStates.SHOOT_AT_OUTPOST)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    driverCont, () -> passCalculator.calculateShot().targetHeading())));
    forceOutpostTrigger.onFalse(superStructure.setStateCommand(SuperWantedStates.DEFAULT));

    // TODO: Re-enable superStructure bindings
    Trigger intakeTrigger = driverCont.leftTrigger().and(isSuperstructureMode);
    intakeTrigger.onTrue(superStructure.setStateCommand(SuperWantedStates.INTAKING));
    intakeTrigger.onFalse(superStructure.setStateCommand(SuperWantedStates.DEFAULT));

    driverCont
        .a()
        .and(isSuperstructureMode)
        .onTrue(superStructure.setStateCommand(SuperWantedStates.UNJAMMING));
    driverCont
        .a()
        .and(isSuperstructureMode)
        .onFalse(superStructure.setStateCommand(SuperWantedStates.DEFAULT));

    configureIndependentModeBindings(isIndependentMode);

    // Drivetrain commands
    // driverCont.leftTrigger().whileTrue(drivetrain.faceHubWhileDriving(driverCont));
    drivetrain.registerTelemetry(logger::telemeterize);
    driverCont.back().onTrue(drivetrain.zeroCommand());
  }

  /** Configures bindings that are active only in independent (test) mode. */
  private void configureIndependentModeBindings(BooleanSupplier isIndependentMode) {
    driverCont.leftBumper().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(0.2));

    driverCont.a().and(isIndependentMode).whileTrue(indexer.setDutyCycleCommand(0.5));
    driverCont.b().and(isIndependentMode).whileTrue(flywheelKicker.setDutyCycleCommand(0.5));
    driverCont
        .rightBumper()
        .and(isIndependentMode)
        .whileTrue(hopperRoller.setDutyCycleCommand(0.5));

    // hood bindings
    driverCont.pov(0).and(isIndependentMode).whileTrue(intakePivot.setPositionCommand(() -> 0.0));
    driverCont
        .pov(180)
        .and(isIndependentMode)
        .whileTrue(intakePivot.setPositionCommand(() -> 93.0));
    driverCont.pov(90).and(isIndependentMode).whileTrue(flywheel.setVelocityCommand(() -> 2000.0));

    driverCont.start().and(isIndependentMode).onTrue(hood.zero());

    // flywheel bindings
    driverCont.x().and(isIndependentMode).whileTrue(hood.setPositionCommand(0.0));
    driverCont.y().and(isIndependentMode).whileTrue(hood.setPositionCommand(20.0));

    // configureIntakeTestBindings(isIndependentMode);
    // configureFullShootingTestBindings(isIndependentMode);
    // configureHoodTestBindings(isIndependentMode);
    // configureClimberTestBindings(isIndependentMode);
  }

  void configureHoodTestBindings(BooleanSupplier isIndependentMode) {
    driverCont.a().and(isIndependentMode).whileTrue(hood.setDutyCycleCommand(0.1));
    driverCont.b().and(isIndependentMode).whileTrue(hood.setDutyCycleCommand(-0.1));
    driverCont.y().and(isIndependentMode).whileTrue(hood.zero());
  }

  /** Configures full intake to shooting test bindings for independent mode. */
  private void configureFullShootingTestBindings() {
    BooleanSupplier isSuperstructureMode =
        () -> superStructure.getControlState() == ControlState.SUPERSTRUCTURE;
    BooleanSupplier isIndependentMode =
        () -> superStructure.getControlState() == ControlState.INDEPENDENT;

    driverCont.rightTrigger().and(isIndependentMode).whileTrue(flywheel.setVelocityCommand(4000));
    driverCont
        .a()
        .and(isIndependentMode)
        .whileTrue(
            indexer
                .setDutyCycleCommand(() -> 0.75)
                .alongWith(
                    hopperRoller.setDutyCycleCommand(0.75),
                    flywheelKicker.setVelocityCommand(4000.0)));
    driverCont
        .b()
        .and(isIndependentMode)
        .whileTrue(
            indexer
                .setDutyCycleCommand(() -> -0.3)
                .alongWith(
                    hopperRoller.setDutyCycleCommand(-0.3),
                    flywheelKicker.setDutyCycleCommand(-0.3),
                    intake.setDutyCycleCommand(-0.2)));
    driverCont.x().and(isIndependentMode).whileTrue(intakePivot.setPositionCommand(() -> 93.0));
    driverCont.y().and(isIndependentMode).whileTrue(intakePivot.setPositionCommand(() -> 0.0));
    driverCont.leftTrigger().and(isIndependentMode).whileTrue(intake.setVelocityCommand(1000.0));
    driverCont.pov(0).and(isIndependentMode).whileTrue(hood.setPositionCommand(0.0));
    driverCont.pov(90).and(isIndependentMode).whileTrue(hood.setPositionCommand(15.0));
    driverCont.pov(180).and(isIndependentMode).whileTrue(hood.setPositionCommand(30.0));
    driverCont.pov(270).and(isIndependentMode).whileTrue(hood.setPositionCommand(40.0));

    driverCont.back().onTrue(drivetrain.zeroCommand());

    // intake stuff
    // driverCont
    //     .axisMagnitudeGreaterThan(5, 0.1)
    //     .and(isIndependentMode)
    //     .whileTrue(intakePivot.setDutyCycleCommand(() -> -driverCont.getRightY() * 0.2));

    // driverCont
    //     .rightBumper()
    //     .and(isIndependentMode)
    //     .whileTrue(intakePivot.setPositionCommand(() -> 0.0));
    // driverCont
    //     .leftBumper()
    //     .and(isIndependentMode)
    //     .whileTrue(intakePivot.setPositionCommand(() -> 90.0));
    // Intake rollers: A = in, B = out
    // driverCont.a().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(0.2));
    // driverCont.b().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(-0.2));
  }

  /** Configures climber test bindings for independent mode. */
  private void configureClimberTestBindings(BooleanSupplier isIndependentMode) {
    driverCont.x().and(isIndependentMode).whileTrue(climber.setLeftDutyCycleCommand(0.1));
    driverCont.y().and(isIndependentMode).whileTrue(climber.setRightDutyCycleCommand(0.1));
  }

  /** Configures intake and intake pivot test bindings for independent mode. */
  private void configureIntakeTestBindings(BooleanSupplier isIndependentMode) {
    // Intake pivot: right joystick Y axis controls duty cycle
    driverCont
        .axisMagnitudeGreaterThan(5, 0.1)
        .and(isIndependentMode)
        .whileTrue(intakePivot.setDutyCycleCommand(() -> -driverCont.getRightY() * 0.2));

    driverCont
        .rightBumper()
        .and(isIndependentMode)
        .whileTrue(intakePivot.setPositionCommand(() -> 0.0));
    driverCont
        .leftBumper()
        .and(isIndependentMode)
        .whileTrue(intakePivot.setPositionCommand(() -> 90.0));
    // Intake rollers: A = in, B = out
    driverCont.a().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(0.2));
    driverCont.b().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(-0.2));
  }

  /** Stops all subsystems safely when the robot is disabled. */
  public void onDisable() {
    superStructure.setControlState(ControlState.SUPERSTRUCTURE);
    superStructure.setWantedSuperState(SuperWantedStates.IDLE);
    drivetrain.setControl(new SwerveRequest.Idle());
    climber.stop();
    flywheel.stop();
    hood.stop();
    intake.stop();
    intakePivot.stop();
    indexer.stop();
    flywheelKicker.stop();
    vision.enableIMUSeeding();
    vision.setThrottle(DISABLED_THROTTLE_SKIP_FRAMES);
  }

  /** Called when the robot transitions from disabled to an enabled mode. */
  public void onEnable() {
    // Ensures superstructure control mode is active when enabled
    superStructure.setControlState(ControlState.SUPERSTRUCTURE);
    superStructure.setWantedSuperState(SuperWantedStates.DEFAULT);
    onEnableVision();
  }

  /** Sets up vision to run at full performance and temperature */
  private void onEnableVision() {
    vision.setThrottle(ENABLED_THROTTLE_SKIP_FRAMES);
  }

  /** Decouples the superstructure from subsystems for test mode. */
  public void onTestEnable() {
    superStructure.setControlState(ControlState.INDEPENDENT);
    superStructure.setWantedSuperState(SuperWantedStates.IDLE);
    onEnableVision();
  }

  /**
   * Pre-computes cached values (e.g. shot parameters) before the command scheduler runs. Must be
   * called in {@link Robot#robotPeriodic()} before {@link
   * edu.wpi.first.wpilibj2.command.CommandScheduler#run()}.
   */
  public void preSchedulerUpdate() {
    hubShotCalculator.clearShootingParams();
    // hubShotCalculator.calculateShot();
    passCalculator.clearShootingParams();
    // passCalculator.calculateShot();
  }

  /**
   * Flushes NetworkTables after the command scheduler runs. This ensures all values written during
   * subsystem periodic methods (e.g., robot orientation for Limelights) are sent in a single batch.
   * Required for all Limelights (not just LL4) because {@link
   * VisionIOLimelightBase#updateInputs(VisionIO.VisionIOInputs)} uses {@code
   * SetRobotOrientation_NoFlush}. Must be called in {@link Robot#robotPeriodic()} after {@link
   * edu.wpi.first.wpilibj2.command.CommandScheduler#run()}.
   */
  public void postSchedulerUpdate() {
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
