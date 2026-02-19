// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtAprilTag extends Command {
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
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
    double xSpeed = 0;
    double ySpeed = 0;
    double turnOutput = 0;

    if (m_vision.hasTarget()) {
      double tx = m_vision.getTx();
      turnOutput = m_turnController.calculate(tx, 0);
    }

    // Use setControl to move the robot
    m_drive.setControl(
        driveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(turnOutput));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot by sending 0 velocities
    m_drive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnController.atSetpoint();
  }
}
