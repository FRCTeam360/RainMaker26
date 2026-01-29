// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.CommandLogger;

/** Add your docs here. */
public class CommandFactory {

  private final Intake intake;
  private final Flywheel flywheel;
  private final FlywheelKicker flyWheelKicker;
  private final Hood hood;
  private final Indexer indexer;
  private final IntakePivot intakePivot;
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;

  // Contructor
  public CommandFactory(
      Intake intake,
      Flywheel flywheel,
      FlywheelKicker flyWheelKicker,
      Hood hood,
      Indexer indexer,
      IntakePivot intakePivot,
      Vision vision,
      CommandSwerveDrivetrain drivetrain) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.flyWheelKicker = flyWheelKicker;
    this.hood = hood;
    this.indexer = indexer;
    this.intakePivot = intakePivot;
    this.vision = vision;
    this.drivetrain = drivetrain;
  }

  public Command basicIntakeCmd() {
    /*return CommandLogger.logCommand(
        SmartIntake.newCommand(coralShooter, funnel)
            .andThen(this.intakeRumble(driverCont).withTimeout(0.2)),
        "Smart Intake with Rumble");*/
    return CommandLogger.logCommand(intake.newCommand(intake)
      .andThen(this.setDutyCycleCommand(0.65))
      .andThen(this.alongWith(flyWheelKicker.setDutyCycleCommand(1.0)))
      .andThen(this.alongWith(indexer.setDutyCycleCommand(0.5))), "Intake");
  }

  public Command basicShootCmd() {
    return flywheel.setDutyCycleCommand(0.75);
  }
}
