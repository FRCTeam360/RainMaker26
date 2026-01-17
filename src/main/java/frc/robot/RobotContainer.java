// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.WoodbotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelIOWB;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Hood.HoodIOWB;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOWB;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOWB;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.IntakePivot.IntakePivotIO;
import frc.robot.subsystems.IntakePivot.IntakePivotIOWB;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final CommandSwerveDrivetrain drivetrain;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Indexer indexer;
  private final Intake intake;
  private final IntakePivot intakePivot;

  //TODO: refactor to allow for more than 1 drivetrain type 

  private Telemetry logger = new Telemetry(WoodbotConstants.kSpeedAt12Volts.in(MetersPerSecond));

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverCont = new CommandXboxController(0);

  private final CommandXboxController testCont1 = new CommandXboxController(5);
  private final CommandXboxController testCont2 = new CommandXboxController(6);
  //private final CommandXboxController operatorCont = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //  switch (Constants.getRobotType()) {
    //   case WOODBOT:
        drivetrain = WoodbotConstants.createDrivetrain();
        flywheel = new Flywheel(new FlywheelIOWB() );
        hood = new Hood(new HoodIOWB());
        indexer = new Indexer(new IndexerIOWB());
        intake = new Intake(new IntakeIOWB());
        intakePivot = new IntakePivot(new IntakePivotIOWB());
    //     break;
    //  }
    // Configure the trigger bindings
    configureBindings();
    configureTestBindings1();
    configureTestBindings2();
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
    System.out.println("fieldoriented drive calleddd");
    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(driverCont));

  }

  private void configureTestBindings1() {
    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(testCont1));
  }

  private void configureTestBindings2() {
    drivetrain.setDefaultCommand(drivetrain.fieldOrientedDrive(testCont2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
