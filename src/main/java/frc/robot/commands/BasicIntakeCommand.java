// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicIntakeCommand extends Command {
  public Intake intake;
  public Indexer indexer;
  private FlywheelKicker kicker;

  /** Creates a new BasicIntakeCommand. */
  public BasicIntakeCommand(Intake intake, Indexer indexer, FlywheelKicker kicker) {
    this.intake = intake;
    this.indexer = indexer;
    this.kicker = kicker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.setDutyCycle(-0.65);
    // indexer.setDutyCycle(0.5);
    kicker.setDutyCycle(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
    kicker.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
