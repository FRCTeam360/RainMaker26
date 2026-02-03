// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.WoodBotDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStates;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIOWB;
import frc.robot.subsystems.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOWB;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOSim;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIOWB;
import frc.robot.subsystems.Hood.HoodIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivotIOSim;
import java.util.Objects;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> autoChooser;
  private CommandSwerveDrivetrain drivetrain;
  private Flywheel flywheel;
  private Hood hood;
  private Indexer indexer;
  private Intake intake;
  private IntakePivot intakePivot;
  private FlywheelKicker flywheelKicker;
  //private CommandFactory commandFactory;

  private final SuperStructure superStructure;

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
        // hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        hood = new Hood(new HoodIOWB());
        // intakePivot = new IntakePivot(new IntakePivotIOPB());
        break;
      case SIM:
        drivetrain = WoodBotDrivetrain.createDrivetrain();
        logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOSim());
        flywheel = new Flywheel(new  FlywheelIOSim());
        hood = new Hood(new HoodIOSim());


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
        // hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        intake = new Intake(new IntakeIOWB());
        flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
        hood = new Hood(new HoodIOWB());
        // intakePivot = new IntakePivot(new IntakePivotIOPB());
    }
    // commandFactory =
    //     new CommandFactory(
    //         intake, flywheel, flywheelKicker, hood, indexer, intakePivot, null, drivetrain);
    // Configure the trigger bindings
    superStructure = new SuperStructure(intake, indexer, flywheelKicker, flywheel, hood);
    
    registerPathplannerCommand("basic intake", superStructure.setStateCommand(SuperStates.INTAKING));
    registerPathplannerCommand("shoot at hub", superStructure.setStateCommand(SuperStates.SPINUP_SHOOTING));
    registerPathplannerCommand("run flywheel kicker", flywheelKicker.setDutyCycleCommand(1.0));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    FollowPathCommand.warmupCommand().schedule();
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
    // TODO: make more elegant solution for null checking subsystems/commands
    // if (Objects.nonNull(intake) && Objects.nonNull(flywheelKicker) && Objects.nonNull(indexer)) {
    //   driverCont.leftBumper().whileTrue(commandFactory.basicIntakeCmd());
    // }
    
    driverCont.leftBumper().onTrue(superStructure.setStateCommand(SuperStates.INTAKING));
    driverCont.leftBumper().onFalse(superStructure.setStateCommand(SuperStates.IDLE));
    
    driverCont.rightBumper().onTrue(superStructure.setStateCommand(SuperStates.SPINUP_SHOOTING));
    driverCont.rightBumper().onFalse(superStructure.setStateCommand(SuperStates.IDLE));
    // if (Objects.nonNull(flywheel)) {
    //   driverCont.rightBumper().whileTrue(commandFactory.basicShootCmd());
    // }

    // setHoodPosition uses hood
    // if (Objects.nonNull(hood)) {
    //   driverCont.pov(0).onTrue(commandFactory.setHoodPosition(0.0));
    //   driverCont.pov(90).onTrue(commandFactory.setHoodPosition(4.0));
    //   driverCont.pov(180).onTrue(commandFactory.setHoodPosition(16.0));
    //   driverCont.pov(270).onTrue(commandFactory.setHoodPosition(23.0));
    //   driverCont.start().onTrue(hood.zero());
    // }

    // shootWithRPM uses flywheel
    // if (Objects.nonNull(flywheel)) {
    //   driverCont.a().whileTrue(commandFactory.shootWithRPM(2000));
    //   driverCont.x().whileTrue(commandFactory.shootWithRPM(2500));
    //   driverCont.b().whileTrue(commandFactory.shootWithRPM(3000));
    //   driverCont.y().whileTrue(commandFactory.shootWithRPM(3500));
    // }

    // Drivetrain commands
    if (Objects.nonNull(drivetrain)) {
      drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(driverCont));
      drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  public void onDisable() {
    if (Objects.nonNull(flywheel)) {
      flywheel.stop();
    }
    if (Objects.nonNull(intake)) {
      intake.stop();
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
    return null;
  }
}
