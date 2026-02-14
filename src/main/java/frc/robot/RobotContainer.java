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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.WoodBotDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOSim;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOWB;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivotIOSim;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOWB;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOWB;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStates;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Vision.VisionIOPhotonSim;
import java.util.Map;
import java.util.Objects;
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
  private FlywheelKicker flywheelKicker;

  private CommandFactory commandFactory;
  private SuperStructure superStructure;

  private ShotCalculator shotCalculator;

  // TODO: refactor to allow for more than 1 drivetrain type

  private Telemetry logger;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverCont = new CommandXboxController(0);

  private final CommandXboxController testCont1 = new CommandXboxController(5);

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
        break;
      case WOODBOT:
      default:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOWB());
        hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        vision =
            new Vision(
                Map.ofEntries(
                    Map.entry(
                        Constants.WoodBotConstants.LIMELIGHT,
                        new VisionIOLimelight(
                            Constants.WoodBotConstants.LIMELIGHT,
                            () -> drivetrain.getAngle(),
                            () -> drivetrain.getAngularRate(),
                            true))));
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        // intakePivot = new IntakePivot(new IntakePivotIOPB());
    }
    shotCalculator = new ShotCalculator(drivetrain);
    // Configure the trigger bindings
    commandFactory =
        new CommandFactory(
            intake, flywheel, flywheelKicker, hood, indexer, intakePivot, vision, drivetrain);
    superStructure =
        new SuperStructure(
            intake, indexer, flywheelKicker, flywheel, hood, drivetrain, shotCalculator);

    if (Objects.nonNull(superStructure)) {
      registerPathplannerCommand(
          "basic intake", superStructure.setStateCommand(SuperStates.INTAKING));
      registerPathplannerCommand(
          "shoot at hub", superStructure.setStateCommand(SuperStates.SHOOTING));
    }
    registerPathplannerCommand("run flywheel kicker", flywheelKicker.setVelocityCommand(4000.0));
    registerPathplannerCommand("spinup flywheel hub shot", commandFactory.shootWithRPM(3000.0));
    configureBindings();
    configureTestBindings();

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    FollowPathCommand.warmupCommand().schedule();

    new JoystickButton(drivetrainController, XboxController.Button.kX.value)
      .whileTrue(new AimAtApriltag(drivetrain, vision));
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
  private void configureTestBindings() {
    if (Objects.nonNull(drivetrain)) {
      drivetrain.setDefaultCommand(
          drivetrain
              .fieldOrientedDrive(testCont1)
              .alongWith(
                  Commands.run(
                      () -> {
                        shotCalculator.calculateShot();
                        shotCalculator.clearShootingParams();
                      })));
      testCont1.rightTrigger().whileTrue(drivetrain.faceHubWhileDriving(testCont1));
      drivetrain.registerTelemetry(logger::telemeterize);
    }

    if (Objects.nonNull(flywheel)) {
      testCont1.a().whileTrue(flywheel.setDutyCycleCommand(() -> 0.5));
    }
    if (Objects.nonNull(flywheelKicker)) {
      testCont1.b().whileTrue(flywheelKicker.setDutyCycleCommand(() -> 0.5));
    }
    if (Objects.nonNull(hood)) {
      testCont1.x().whileTrue(hood.setDutyCycleCommand(() -> 0.5));
    }
    if (Objects.nonNull(indexer)) {
      testCont1.y().whileTrue(indexer.setDutyCycleCommand(() -> 0.5));
    }
    if (Objects.nonNull(intake)) {
      testCont1.leftBumper().whileTrue(intake.setDutyCycleCommand(() -> 0.5));
    }
    if (Objects.nonNull(intakePivot)) {
      testCont1.rightBumper().whileTrue(intakePivot.setDutyCycleCommand(() -> 0.5));
    }
  }

  private void configureBindings() {
    // Only bind commands if the required subsystems/factories exist
    if (Objects.nonNull(vision)) {
      Command consumeVisionMeasurements =
          vision.consumeVisionMeasurements(
              measurements -> {
                drivetrain.addVisionMeasurements(measurements);
              });
      vision.setDefaultCommand(consumeVisionMeasurements.ignoringDisable(true));
    }
    // TODO: make more elegant solution for null checking subsystems/commands

    // Null checks based on subsystems used by each command
    // basicIntakeCmd uses intake and indexer
    if (Objects.nonNull(intake) && Objects.nonNull(indexer) && Objects.nonNull(superStructure)) {
      driverCont.leftBumper().onTrue(superStructure.setStateCommand(SuperStates.INTAKING));
      driverCont.leftBumper().onFalse(superStructure.setStateCommand(SuperStates.IDLE));
    }

    // setFlywheelKickerDutyCycle uses flywheelKicker
    if (Objects.nonNull(flywheelKicker)) {
      driverCont.rightBumper().whileTrue(flywheelKicker.setVelocityCommand(4000.0));
    }

    // setHoodPosition uses hood
    if (Objects.nonNull(hood)) {
      driverCont.pov(0).onTrue(commandFactory.setHoodPosition(0.0));
      driverCont.pov(90).onTrue(commandFactory.setHoodPosition(4.0));
      driverCont.pov(180).onTrue(commandFactory.setHoodPosition(16.0));
      driverCont.pov(270).onTrue(commandFactory.setHoodPosition(23.0));
      driverCont.start().onTrue(hood.zero());
    }

    // shootWithRPM uses flywheel
    if (Objects.nonNull(flywheel)) {
      driverCont.a().whileTrue(commandFactory.shootWithRPM(2000));
      driverCont.x().whileTrue(commandFactory.shootWithRPM(2500));
      driverCont.b().whileTrue(commandFactory.shootWithRPM(3000));
      driverCont.y().whileTrue(commandFactory.shootWithRPM(3500));
      // driverCont.rightTrigger().whileTrue(commandFactory.shootWithSpinUp(3500.0, 6.0));
      if (Objects.nonNull(superStructure)) {
        driverCont.rightTrigger().onTrue(superStructure.setStateCommand(SuperStates.SHOOTING));
        driverCont.rightTrigger().onFalse(superStructure.setStateCommand(SuperStates.IDLE));
      }
    }

    // Drivetrain commands
    if (Objects.nonNull(drivetrain)) {
      drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(driverCont));
      driverCont.leftTrigger().whileTrue(drivetrain.faceHubWhileDriving(driverCont));
      drivetrain.registerTelemetry(logger::telemeterize);
      driverCont.back().onTrue(drivetrain.zeroCommand());
    }
  }

  /** Stops all subsystems safely when the robot is disabled. */
  public void onDisable() {
    if (Objects.nonNull(superStructure)) superStructure.setWantedSuperState(SuperStates.IDLE);
    if (Objects.nonNull(drivetrain)) {
      drivetrain.setControl(new SwerveRequest.Idle());
    }
    if (Objects.nonNull(flywheel)) {
      flywheel.stop();
    }
    if (Objects.nonNull(hood)) {
      hood.stop();
    }
    if (Objects.nonNull(intake)) {
      intake.stop();
    }
    if (Objects.nonNull(intakePivot)) {
      intakePivot.stop();
    }
    if (Objects.nonNull(indexer)) {
      indexer.stop();
    }
    if (Objects.nonNull(flywheelKicker)) {
      flywheelKicker.stop();
    }
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
