package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;
import org.littletonrobotics.junction.Logger;

public class PIDToPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d setpointPose;
  private final String LOGGING_PREFIX = "PIDToPose: ";

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
    final boolean isAtHeadingSetpoint = drivetrain.isAtRotationSetpoint();
    final boolean isAtPoseXSetpoint = drivetrain.isAtPoseXSetpoint();
    final boolean isAtPoseYSetpoint = drivetrain.isAtPoseYSetpoint();

    final double headingPositionError =
        Math.toDegrees(drivetrain.getHeadingControllerPositionError());
    final double poseXPositionError = drivetrain.getPoseXControllerPositionError();
    final double poseYPositionError = drivetrain.getPoseYControllerPositionError();

    final double headingVelocityError =
        Math.toDegrees(drivetrain.getHeadingControllerVelocityError());
    final double poseXVelocityError = drivetrain.getPoseXControllerVelocityError();
    final double poseYVelocityError = drivetrain.getPoseYControllerVelocityError();

    Logger.recordOutput(LOGGING_PREFIX + "isAtRotationSetpoint", isAtHeadingSetpoint);
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseXSetpoint", isAtPoseXSetpoint);
    Logger.recordOutput(LOGGING_PREFIX + "isAtPoseYSetpoint", isAtPoseYSetpoint);

    Logger.recordOutput(LOGGING_PREFIX + "headingPositionError", headingPositionError);
    Logger.recordOutput(LOGGING_PREFIX + "poseXPositionError", poseXPositionError);
    Logger.recordOutput(LOGGING_PREFIX + "poseYPositionError", poseYPositionError);

    Logger.recordOutput(LOGGING_PREFIX + "headingVelocityError", headingVelocityError);
    Logger.recordOutput(LOGGING_PREFIX + "poseXVelocityError", poseXVelocityError);
    Logger.recordOutput(LOGGING_PREFIX + "poseYVelocityError", poseYVelocityError);

    return isAtHeadingSetpoint && isAtPoseXSetpoint && isAtPoseYSetpoint;
  }

  public static Command getCommand(CommandSwerveDrivetrain drivetrain, Pose2d setpointPose) {
    return CommandLogger.logCommand(new PIDToPose(drivetrain, setpointPose), "DriveToPose");
  }
}
