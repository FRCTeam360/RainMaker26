package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that faces the target while driving and x-outs the wheels when the robot is stationary
 * and aligned. If the driver provides stick input or the heading drifts off target, the command
 * exits x-out and returns to face-angle driving to re-align.
 *
 * <p>Uses the heading controller's {@code atSetpoint()} for the entry check (valid while face-angle
 * is active), and manual heading comparison for the exit check (reliable while x-outed since the
 * heading controller is stale).
 */
public class XOutWhileShootingCommand extends Command {
  private static final String LOG_PREFIX = "Swerve/XOutShooting/";

  /** Minimum commanded velocity (fraction of max speed) to count as driver input. */
  private static final double DRIVER_INPUT_THRESHOLD = 0.05;

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  private boolean isXOuted = false;
  private boolean stillAligned = false;

  /**
   * Creates a new XOutWhileShootingCommand.
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param velocityXSupplier Supplier for X velocity (field-relative, forward positive) in m/s
   * @param velocityYSupplier Supplier for Y velocity (field-relative, left positive) in m/s
   * @param headingSupplier Supplier for the target heading to face
   */
  public XOutWhileShootingCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      Supplier<Rotation2d> headingSupplier) {
    this.drivetrain = drivetrain;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
    this.headingSupplier = headingSupplier;
    addRequirements(drivetrain);
  }

  /**
   * Creates a new XOutWhileShootingCommand using controller input with cubic response curve.
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param driveCont The Xbox controller for driver input
   * @param headingSupplier Supplier for the target heading to face
   */
  public XOutWhileShootingCommand(
      CommandSwerveDrivetrain drivetrain,
      CommandXboxController driveCont,
      Supplier<Rotation2d> headingSupplier) {
    this(
        drivetrain,
        () ->
            Math.pow(driveCont.getLeftY(), 3)
                * CommandSwerveDrivetrain.maxSpeed.in(MetersPerSecond)
                * -1.0,
        () ->
            Math.pow(driveCont.getLeftX(), 3)
                * CommandSwerveDrivetrain.maxSpeed.in(MetersPerSecond)
                * -1.0,
        headingSupplier);
  }

  @Override
  public void initialize() {
    isXOuted = false;
    stillAligned = false;
  }

  @Override
  public void execute() {
    double vx = velocityXSupplier.getAsDouble();
    double vy = velocityYSupplier.getAsDouble();
    double commandedSpeedMps = Math.hypot(vx, vy);
    boolean hasDriverInput =
        commandedSpeedMps
            > CommandSwerveDrivetrain.maxSpeed.in(MetersPerSecond) * DRIVER_INPUT_THRESHOLD;

    if (isXOuted) {
      // Heading controller is stale while x-outed, so check alignment manually
      double headingErrorRad = headingSupplier.get().minus(drivetrain.getRotation2d()).getRadians();
      stillAligned = Math.abs(headingErrorRad) < drivetrain.getHeadingToleranceRad();

      if (hasDriverInput || !stillAligned) {
        isXOuted = false;
        stillAligned = false;
        drivetrain.resetHeadingController();
      } else {
        drivetrain.xOut();
      }
    }

    if (!isXOuted) {
      // While face-angle is active, isAlignedToTarget() is valid
      if (!hasDriverInput && drivetrain.isAlignedToTarget()) {
        isXOuted = true;
        drivetrain.xOut();
      } else {
        Rotation2d target = headingSupplier.get();
        drivetrain.faceAngleWhileDriving(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? vx : -vx,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? vy : -vy,
            target);
      }
    }

    Logger.recordOutput(LOG_PREFIX + "HasDriverInput", hasDriverInput);
    Logger.recordOutput(LOG_PREFIX + "IsXOuted", isXOuted);
    Logger.recordOutput(LOG_PREFIX + "StillAligned", stillAligned);
  }

  @Override
  public void end(boolean interrupted) {
    isXOuted = false;
    stillAligned = false;
    drivetrain.resetHeadingController();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Returns whether the robot is currently x-outed (wheels locked).
   *
   * @return true if x-outed
   */
  public boolean isXOuted() {
    return isXOuted;
  }

  /**
   * Returns whether the robot is still aligned to the target while x-outed.
   *
   * @return true if aligned (either via heading controller or manual check while x-outed)
   */
  public boolean isStillAligned() {
    return stillAligned;
  }
}
