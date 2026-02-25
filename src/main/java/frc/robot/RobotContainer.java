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
import frc.robot.generated.WoodBotDrivetrain;
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
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOWB;
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
import frc.robot.subsystems.SuperStructure.SuperStates;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight3G;
import frc.robot.subsystems.Vision.VisionIOLimelight4;
import frc.robot.subsystems.Vision.VisionIOPhotonSim;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
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

  private SuperStructure superStructure;

  private ShotCalculator hubShotCalculator;
  private ShotCalculator outpostPassCalculator;

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
                ShooterConstants.ROBOT_TO_SHOOTER,
                0.0,
                5.0);
        break;
      case WOODBOT:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOWB());
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
                ShooterConstants.ROBOT_TO_SHOOTER,
                0.0,
                5.0);
        break;
      case PRACTICEBOT:
      default:
        drivetrain =
            WoodBotDrivetrain
                .createDrivetrain(); // FIXME, CHANGE ONCE PRACTICE BOT DRIVETRAIN IS MADE
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOPB());
        hood = new Hood(new HoodIOPB());
        indexer = new Indexer(new IndexerIOPB());
        vision = // TODO ADD OTHER LIMELIGHTS
            new Vision(
                Map.ofEntries(
                    Map.entry(
                        Constants.PracticeBotConstants.LIMELIGHT,
                        new VisionIOLimelight3G(
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
                Constants.WoodBotConstants.shotHoodAngleMap,
                Constants.WoodBotConstants.shotFlywheelSpeedMap,
                Constants.WoodBotConstants.timeOfFlightMap,
                ShooterConstants.ROBOT_TO_SHOOTER,
                0.0,
                5.0);
        // TODO ADD CLIMBERS
        break;
    }
    hubShotCalculator =
        new ShotCalculator(
            drivetrain::getPosition,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
            robotShootingInfo);
    outpostPassCalculator =
        new ShotCalculator(
            drivetrain::getPosition,
            () -> AllianceFlipUtil.apply(FieldConstants.Outpost.centerPoint),
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
            outpostPassCalculator,
            drivetrain::isAlignedToTarget);

    registerPathplannerCommand(
        "basic intake", superStructure.setStateCommand(SuperStates.INTAKING));
    // TODO: add end condition based on state from SuperStructure (based on sensor inputs)
    registerPathplannerCommand(
        "shoot at hub",
        Commands.waitSeconds(10)
            .deadlineFor(
                superStructure
                    .setStateCommand(SuperStates.SHOOT_AT_HUB)
                    .alongWith(
                        drivetrain.faceAngleWhileDrivingCommand(
                            () -> 0,
                            () -> 0,
                            () -> hubShotCalculator.calculateShot().targetHeading())))
            .andThen(superStructure.setStateCommand(SuperStates.IDLE)));

    configureBindings();
    // configureTestBindings();

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
    Command consumeVisionMeasurements =
        vision.consumeVisionMeasurements(
            measurements -> {
              drivetrain.addVisionMeasurements(measurements);
            });
    vision.setDefaultCommand(consumeVisionMeasurements.ignoringDisable(true));

    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDriveCommand(driverCont));

    BooleanSupplier isSuperstructureMode =
        () -> superStructure.getControlState() == ControlState.SUPERSTRUCTURE;
    BooleanSupplier isIndependentMode =
        () -> superStructure.getControlState() == ControlState.INDEPENDENT;

    // Linked pair: whileTrue sets SHOOTING + aims, onFalse resets to IDLE.
    // The InstantCommand (setStateCommand) finishes immediately; the alongWith group
    // stays alive via faceAngleWhileDrivingCommand until whileTrue interrupts it.
    Trigger shootAtHubTrigger = driverCont.rightTrigger().and(isSuperstructureMode);
    shootAtHubTrigger.whileTrue(
        superStructure
            .setStateCommand(SuperStates.SHOOT_AT_HUB)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    driverCont, () -> hubShotCalculator.calculateShot().targetHeading())));
    // Must stay paired with the whileTrue above to reset state on trigger release
    shootAtHubTrigger.onFalse(superStructure.setStateCommand(SuperStates.IDLE));

    Trigger shootAtOutpostTrigger = driverCont.leftTrigger().and(isSuperstructureMode);
    shootAtOutpostTrigger.whileTrue(
        superStructure
            .setStateCommand(SuperStates.SHOOT_AT_OUTPOST)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    driverCont, () -> outpostPassCalculator.calculateShot().targetHeading())));
    // Must stay paired with the whileTrue above to reset state on trigger release
    shootAtOutpostTrigger.onFalse(superStructure.setStateCommand(SuperStates.IDLE));

    // TODO: Re-enable superStructure bindings
    Trigger intakeTrigger = driverCont.leftBumper().and(isSuperstructureMode);
    intakeTrigger.onTrue(superStructure.setStateCommand(SuperStates.INTAKING));
    intakeTrigger.onFalse(superStructure.setStateCommand(SuperStates.IDLE));

    configureIndependentModeBindings(isIndependentMode);

    // Drivetrain commands
    // driverCont.leftTrigger().whileTrue(drivetrain.faceHubWhileDriving(driverCont));
    drivetrain.registerTelemetry(logger::telemeterize);
    driverCont.back().onTrue(drivetrain.zeroCommand());
  }

  /** Configures bindings that are active only in independent (test) mode. */
  private void configureIndependentModeBindings(BooleanSupplier isIndependentMode) {
    // driverCont.leftBumper().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(0.2));
    //
    // driverCont.a().and(isIndependentMode).whileTrue(indexer.setDutyCycleCommand(0.5));
    //
    // // hood bindings
    // driverCont.pov(0).and(isIndependentMode).onTrue(hood.moveToZeroAndZero());
    // driverCont.pov(90).and(isIndependentMode).whileTrue(hood.setPositionCommand(4.0));
    // driverCont.pov(180).and(isIndependentMode).whileTrue(hood.setPositionCommand(16.0));
    // driverCont.pov(270).and(isIndependentMode).whileTrue(hood.setPositionCommand(23.0));
    // driverCont.start().and(isIndependentMode).onTrue(hood.zero());
    //
    // // flywheel bindings
    // driverCont.x().and(isIndependentMode).whileTrue(flywheel.setVelocityCommand(2500));
    // driverCont.b().and(isIndependentMode).whileTrue(flywheel.setVelocityCommand(3000));
    // driverCont.y().and(isIndependentMode).whileTrue(flywheel.setVelocityCommand(3500));

    // configureIntakeTestBindings(isIndependentMode);
    // configureFullShootingTestBindings(isIndependentMode);
    configureHoodTestBindings(isIndependentMode);
  }

  void configureHoodTestBindings(BooleanSupplier isIndependentMode) {
    driverCont.a().and(isIndependentMode).whileTrue(hood.setDutyCycleCommand(0.1));
    driverCont.b().and(isIndependentMode).whileTrue(hood.setDutyCycleCommand(-0.1));
    driverCont.y().and(isIndependentMode).whileTrue(hood.zero());
  }

  /** Configures full intake to shooting test bindings for independent mode. */
  private void configureFullShootingTestBindings(BooleanSupplier isIndependentMode) {
    driverCont.rightTrigger().and(isIndependentMode).whileTrue(flywheel.setDutyCycleCommand(1.0));
    driverCont
        .a()
        .and(isIndependentMode)
        .whileTrue(
            indexer
                .setDutyCycleCommand(0.75)
                .alongWith(
                    hopperRoller.setDutyCycleCommand(0.75),
                    flywheelKicker.setDutyCycleCommand(0.75)));
    driverCont
        .b()
        .and(isIndependentMode)
        .whileTrue(
            indexer
                .setDutyCycleCommand(-0.2)
                .alongWith(
                    hopperRoller.setDutyCycleCommand(-0.2),
                    flywheelKicker.setDutyCycleCommand(-0.2),
                    intake.setDutyCycleCommand(-0.2)));
    driverCont.x().and(isIndependentMode).onTrue(intakePivot.setPositionCommand(() -> 90.0));
    driverCont.y().and(isIndependentMode).onTrue(intakePivot.setPositionCommand(() -> 0.0));
    driverCont.leftTrigger().and(isIndependentMode).whileTrue(intake.setDutyCycleCommand(0.2));
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
    superStructure.setWantedSuperState(SuperStates.IDLE);
    drivetrain.setControl(new SwerveRequest.Idle());
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
    onEnableVision();
  }

  /** Sets up vision to run at full performance and temperature */
  private void onEnableVision() {
    vision.enableIMUAssist();
    vision.setThrottle(ENABLED_THROTTLE_SKIP_FRAMES);
  }

  /** Decouples the superstructure from subsystems for test mode. */
  public void onTestEnable() {
    superStructure.setControlState(ControlState.INDEPENDENT);
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
    outpostPassCalculator.clearShootingParams();
    // outpostPassCalculator.calculateShot();
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
