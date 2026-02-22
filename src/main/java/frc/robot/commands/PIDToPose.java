package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;
import org.littletonrobotics.junction.Logger;

public class PIDToPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d setpointPose;
  private static final String LOGGING_PREFIX = "PIDToPose: ";

  /** Creates a new PIDToPose. */
  public PIDToPose(CommandSwerveDrivetrain drivetrain, Pose2d setpointPose) {
    this.setpointPose = setpointPose;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    Logger.recordOutput(LOGGING_PREFIX + "isAtRotationSetpoint", false);
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseXSetpoint", false);
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseYSetpoint", false);

    Logger.recordOutput(LOGGING_PREFIX + "headingSetpoint", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseXSetpoint", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseYSetpoint", 0.0);

    Logger.recordOutput(LOGGING_PREFIX + "headingPositionError", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseXPositionError", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseYPositionError", 0.0);

    Logger.recordOutput(LOGGING_PREFIX + "headingVelocityError", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseXVelocityError", 0.0);
    Logger.recordOutput(LOGGING_PREFIX + "poseYVelocityError", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final double headingSetpoint = setpointPose.getRotation().getDegrees();

    Logger.recordOutput(LOGGING_PREFIX + "positionSetpoint", setpointPose);
    Logger.recordOutput(LOGGING_PREFIX + "headingSetpoint", headingSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveToPose(setpointPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.xOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final var headingState = drivetrain.getHeadingControllerState();
    final var poseXState = drivetrain.getPoseXControllerState();
    final var poseYState = drivetrain.getPoseYControllerState();

    Logger.recordOutput(LOGGING_PREFIX + "headingSetpoint", headingState.setpoint());
    Logger.recordOutput(LOGGING_PREFIX + "poseXSetpoint", poseXState.setpoint());
    Logger.recordOutput(LOGGING_PREFIX + "poseYSetpoint", poseYState.setpoint());

    Logger.recordOutput(LOGGING_PREFIX + "isAtRotationSetpoint", headingState.isAtSetpoint());
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseXSetpoint", poseXState.isAtSetpoint());
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseYSetpoint", poseYState.isAtSetpoint());

    Logger.recordOutput(LOGGING_PREFIX + "headingPositionError", headingState.positionError());
    Logger.recordOutput(LOGGING_PREFIX + "poseXPositionError", poseXState.positionError());
    Logger.recordOutput(LOGGING_PREFIX + "poseYPositionError", poseYState.positionError());

    Logger.recordOutput(LOGGING_PREFIX + "headingVelocityError", headingState.velocityError());
    Logger.recordOutput(LOGGING_PREFIX + "poseXVelocityError", poseXState.velocityError());
    Logger.recordOutput(LOGGING_PREFIX + "poseYVelocityError", poseYState.velocityError());

    return headingState.isAtSetpoint() && poseXState.isAtSetpoint() && poseYState.isAtSetpoint();
  }

  public static Command getCommand(CommandSwerveDrivetrain drivetrain, Pose2d setpointPose) {
    return CommandLogger.logCommand(new PIDToPose(drivetrain, setpointPose), "DriveToPose");
  }
}
