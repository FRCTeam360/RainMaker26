// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicIntakeCommand;
import frc.robot.generated.WoodBotDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStates;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIOWB;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.FlywheelKicker.FlywheelKickerIOWB;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.IntakePivot.IntakePivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private CommandSwerveDrivetrain drivetrain;
  private Flywheel flywheel;
  private Hood hood;
  private Indexer indexer;
  private Intake intake;
  private IntakePivot intakePivot;
  private FlywheelKicker flywheelKicker;

  private final SuperStructure superStructure;

  // TODO: refactor to allow for more than 1 drivetrain type

  private Telemetry logger = new Telemetry(WoodBotDrivetrain.kSpeedAt12Volts.in(MetersPerSecond));

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverCont = new CommandXboxController(0);

  private final CommandXboxController testCont1 = new CommandXboxController(4);
  private final CommandXboxController testCont2 = new CommandXboxController(5);

  private BasicIntakeCommand basicIntakeCommand;

  // private final CommandXboxController operatorCont = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // switch (Constants.getRobotType()) {
    // case WOODBOT:
    drivetrain = WoodBotDrivetrain.createDrivetrain();
    flywheel = new Flywheel(new FlywheelIOWB());
    // hood = new Hood(new HoodIOWB());
    indexer = new Indexer(new IndexerIOWB());
    intake = new Intake(new IntakeIOWB());
    flywheelKicker = new FlywheelKicker(new FlywheelKickerIOWB());
    // intakePivot = new IntakePivot(new IntakePivotIOPB());
    // break;
    // }
    // Configure the trigger bindings
    superStructure = new SuperStructure(intake, indexer);
    configureBindings();
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
    basicIntakeCommand = new BasicIntakeCommand(intake, indexer);
    driverCont.leftBumper().whileTrue(basicIntakeCommand);
    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(driverCont));
    //driverCont.a().whileTrue(flywheel.setDutyCycleCommand(() -> driverCont.getRightTriggerAxis()));
    driverCont.a()
    .onTrue(superStructure.setStateCommand(SuperStates.COLLECTING_FUEL));
    driverCont.a()
    .onFalse(superStructure.setStateCommand(SuperStates.STOPPED));
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
