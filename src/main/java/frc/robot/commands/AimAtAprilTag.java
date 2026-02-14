// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtAprilTag extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private final Vision m_vision;
  // PID constants need tuning
  private final PIDController m_turnController = new PIDController(0.03, 0.0, 0.0);  
  private static final double kTurnToleranceDeg = 1.0;

  /** Creates a new AimAtAprilTag. */
  public AimAtAprilTag(CommandSwerveDrivetrain drive, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_vision = vision;
    addRequirements(drive, vision);

    // config PID controller for cont input of angles
    m_turnController.enableContinuousInput(-180.0, 180.0);
    m_turnController.setTolerance(kTurnToleranceDeg);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.hasTarget()) {
      double tx = m_vision.getTx();
      // calc motor output using PID controller, input is tx
      double turnOutput = m_turnController.calculate(tx);

      // apply angular velocity and command drivetrain to turn to turnOutput
      m_drive.setTurnSpeed(turnOutput); // all of these r temp until i find out how to use swerve version of this. // TODO fix for swerve
    } else {
      m_drive.setTurnSpeed(0); // TODO fix for swerve
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setTurnSpeed(0); // TODO fix for swerve
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnController.atSetpoint();
  }
}
