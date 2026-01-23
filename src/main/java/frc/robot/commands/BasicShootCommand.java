// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Hood.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicShootCommand extends Command {
  public Flywheel shooter;
  public Hood hood;
  /** Creates a new BasicShootCommand. */
  // TODO: turn this command into an in-line command
  public BasicShootCommand(Flywheel shooter, Hood hood) {
    addRequirements(shooter, hood);
    this.shooter = shooter;
    this.hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setDutyCycle(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
