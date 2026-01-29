// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.WoodBotDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIOWB;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOWB;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIOWB;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.IntakePivot.IntakePivotIOSim;
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
  private Intake intake;
  private IntakePivot intakePivot;
  private FlywheelKicker flywheelKicker;

  private CommandFactory commandFactory;

  // TODO: refactor to allow for more than 1 drivetrain type

  private Telemetry logger;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverCont = new CommandXboxController(0);

  private final CommandXboxController testCont1 = new CommandXboxController(5);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobotType()) {
      case WOODBOT:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOWB());
        hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        // intakePivot = new IntakePivot(new IntakePivotIOPB());
        break;
      case SIM:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        intakePivot = new IntakePivot(new IntakePivotIOSim());

        // flywheel = new Flywheel(new FlywheelIOSim());
        // hood = new Hood(new HoodIOWB());
        // indexer = new Indexer(new IndexerIOSim());
        // intake = new Intake(new IntakeIOSim());
        // flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        break;
      default:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        flywheel = new Flywheel(new FlywheelIOWB());
        hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        // intakePivot = new IntakePivot(new IntakePivotIOPB());
    }
    // Configure the trigger bindings
      registerPathplannerCommand("basic intake", commandFactory.basicIntakeCmd());
      registerPathplannerCommand("shoot at hub", commandFactory.shootWithSpinUp(3000.0, 4.0));
      commandFactory = new CommandFactory(
      intake,
       flywheel,
       flywheelKicker,
       hood,
       indexer,
       drivetrain
  );
    configureBindings();

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    FollowPathCommand.warmupCommand().schedule();
  }

  public void registerPathplannerCommand(String name, Command command) {
    if (Objects.nonNull(command)) {
      NamedCommands.registerCommand(name, command);
    } else {
      System.err.println(name + " is null");
      NamedCommands.registerCommand(name, new InstantCommand(() -> System.err.println(name + " is null")));
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
    // TODO: make more elegant solution for null checking subsystems/commands
    driverCont.leftBumper().whileTrue(commandFactory.basicIntakeCmd());
    driverCont.rightBumper().whileTrue(commandFactory.setFlywheelKickerDutyCycle(1.0));
    driverCont.pov(0).onTrue(commandFactory.setHoodPosition(0.0));
    driverCont.pov(90).onTrue(commandFactory.setHoodPosition(4.0));
    driverCont.pov(180).onTrue(commandFactory.setHoodPosition(16.0));
    driverCont.pov(270).onTrue(commandFactory.setHoodPosition(23.0));
    driverCont.a().whileTrue(commandFactory.shootWithRPM(2000));
    driverCont.x().whileTrue(commandFactory.shootWithRPM(2500));
    driverCont.b().whileTrue(commandFactory.shootWithRPM(3000));
    driverCont.y().whileTrue(commandFactory.shootWithRPM(3500));
    if (Objects.nonNull(drivetrain)) {
      drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(driverCont));
    }
    driverCont.start().onTrue(hood.zero());

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Stops all subsystems safely when the robot is disabled. */
  public void onDisable() {
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
