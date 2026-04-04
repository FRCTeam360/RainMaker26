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
 * Command that faces a target heading while driving, then x-outs the wheels once the robot is
 * stationary and aligned. If the driver provides stick input or the heading drifts off target, the
 * command exits x-out and returns to face-angle driving to re-align.
 *
 * <p>Uses the heading controller's {@code isAlignedToTarget()} for the entry check (valid while
 * face-angle is active), and manual heading comparison for the exit check (reliable while x-outed
 * since the heading controller is stale).
 */
public class XOutWhileAligningCommand extends Command {
  private static final String LOG_PREFIX = "Swerve/XOutAligning/";

  /** Minimum commanded velocity (fraction of max speed) to count as driver input. */
  private static final double DRIVER_INPUT_THRESHOLD = 0.05;

  private enum State {
    FACING_ANGLE,
    X_OUT
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  private State state;

  /**
   * Creates a new XOutWhileAligningCommand.
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param velocityXSupplier Supplier for X velocity (operator-perspective, forward positive) in
   *     m/s
   * @param velocityYSupplier Supplier for Y velocity (operator-perspective, left positive) in m/s
   * @param headingSupplier Supplier for the target heading to face
   */
  public XOutWhileAligningCommand(
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
   * Creates a new XOutWhileAligningCommand using controller input with cubic response curve.
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param driveCont The Xbox controller for driver input
   * @param headingSupplier Supplier for the target heading to face
   */
  public XOutWhileAligningCommand(
      CommandSwerveDrivetrain drivetrain,
      CommandXboxController driveCont,
      Supplier<Rotation2d> headingSupplier) {
    this(
        drivetrain,
        () -> Math.pow(driveCont.getLeftY(), 3) * drivetrain.maxSpeed.in(MetersPerSecond) * -1.0,
        () -> Math.pow(driveCont.getLeftX(), 3) * drivetrain.maxSpeed.in(MetersPerSecond) * -1.0,
        headingSupplier);
  }

  @Override
  public void initialize() {
    state = State.FACING_ANGLE;
    drivetrain.resetHeadingController();
  }

  @Override
  public void execute() {
    double vx = velocityXSupplier.getAsDouble();
    double vy = velocityYSupplier.getAsDouble();
    Rotation2d heading = headingSupplier.get();

    updateState(vx, vy);
    applyState(vx, vy, heading);
  }

  /**
   * Evaluates conditions and transitions to the appropriate state.
   *
   * @param vx sampled X velocity (operator-perspective) in m/s
   * @param vy sampled Y velocity (operator-perspective) in m/s
   */
  private void updateState(double vx, double vy) {
    double commandedSpeedMps = Math.hypot(vx, vy);
    boolean hasDriverInput =
        commandedSpeedMps > drivetrain.maxSpeed.in(MetersPerSecond) * DRIVER_INPUT_THRESHOLD;

    Logger.recordOutput(LOG_PREFIX + "HasDriverInput", hasDriverInput);

    switch (state) {
      case FACING_ANGLE:
        if (!hasDriverInput && drivetrain.isAlignedToTarget()) {
          state = State.X_OUT;
        }
        break;

      case X_OUT:
        double headingErrorRad =
            headingSupplier.get().minus(drivetrain.getRotation2d()).getRadians();
        // TODO: Verify and tune this tolerance on the real robot. Intentionally uses the
        // static (tighter) tolerance rather than the dynamic one because the robot is
        // stationary while x-outed, so there is no speed-based reason to widen it.
        boolean stillAligned = Math.abs(headingErrorRad) < drivetrain.getHeadingToleranceRad();

        if (hasDriverInput || !stillAligned) {
          state = State.FACING_ANGLE;
          drivetrain.resetHeadingController();
        }
        break;
    }

    Logger.recordOutput(LOG_PREFIX + "State", state.name());
  }

  /**
   * Applies the current state to the drivetrain.
   *
   * @param vx sampled X velocity (operator-perspective) in m/s
   * @param vy sampled Y velocity (operator-perspective) in m/s
   * @param heading sampled target heading
   */
  private void applyState(double vx, double vy, Rotation2d heading) {
    switch (state) {
      case FACING_ANGLE:
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        drivetrain.faceAngleWhileDriving(
            isBlueAlliance ? vx : -vx, isBlueAlliance ? vy : -vy, heading);
        break;

      case X_OUT:
        drivetrain.xOut();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
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
    return state == State.X_OUT;
  }
}
