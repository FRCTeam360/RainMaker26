package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

  /** Heading tolerance required to enter X_OUT (tight). */
  private static final double ENTRY_HEADING_TOLERANCE_RAD = Math.toRadians(1.0);

  /** Heading tolerance allowed while in X_OUT before exiting (loose). */
  private static final double EXIT_HEADING_TOLERANCE_RAD = Math.toRadians(3.0);

  /** Maximum measured angular velocity to allow entry into X_OUT (deg/s). */
  private static final double ENTRY_ANGULAR_VELOCITY_TOLERANCE_DPS = 5.0;

  /** Maximum measured translational speed to allow entry into X_OUT (m/s). */
  private static final double ENTRY_LINEAR_VELOCITY_TOLERANCE_MPS = 0.05;

  /** Time the entry condition must be continuously true before transitioning to X_OUT (seconds). */
  private static final double ENTRY_DEBOUNCE_SECONDS = 0.1;

  /** Time the exit condition must be continuously true before leaving X_OUT (seconds). */
  private static final double EXIT_DEBOUNCE_SECONDS = 0.1;

  private enum State {
    FACING_ANGLE,
    X_OUT
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  private State state;
  private final Debouncer entryDebouncer =
      new Debouncer(ENTRY_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Debouncer exitDebouncer = new Debouncer(EXIT_DEBOUNCE_SECONDS, DebounceType.kRising);

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
    entryDebouncer.calculate(false);
    exitDebouncer.calculate(false);
  }

  @Override
  public void execute() {
    double vx = velocityXSupplier.getAsDouble();
    double vy = velocityYSupplier.getAsDouble();
    Rotation2d heading = headingSupplier.get();

    updateState(vx, vy, heading);
    applyState(vx, vy, heading);
  }

  /**
   * Evaluates conditions and transitions to the appropriate state.
   *
   * @param vx sampled X velocity (operator-perspective) in m/s
   * @param vy sampled Y velocity (operator-perspective) in m/s
   * @param heading sampled target heading
   */
  private void updateState(double vx, double vy, Rotation2d heading) {
    double commandedSpeedMps = Math.hypot(vx, vy);
    boolean hasDriverInput =
        commandedSpeedMps > drivetrain.maxSpeed.in(MetersPerSecond) * DRIVER_INPUT_THRESHOLD;

    Logger.recordOutput(LOG_PREFIX + "HasDriverInput", hasDriverInput);

    switch (state) {
      case FACING_ANGLE:
        double entryErrorRad = heading.minus(drivetrain.getRotation2d()).getRadians();
        ChassisSpeeds measured = drivetrain.getVelocity();
        boolean robotStationary =
            Math.abs(drivetrain.getAngularRate()) < ENTRY_ANGULAR_VELOCITY_TOLERANCE_DPS
                && Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond)
                    < ENTRY_LINEAR_VELOCITY_TOLERANCE_MPS;
        boolean headingAligned =
            entryDebouncer.calculate(
                Math.abs(entryErrorRad) < ENTRY_HEADING_TOLERANCE_RAD && robotStationary);
        exitDebouncer.calculate(false);
        if (!hasDriverInput && headingAligned) {
          state = State.X_OUT;
        }
        break;

      case X_OUT:
        double headingErrorRad = heading.minus(drivetrain.getRotation2d()).getRadians();
        boolean headingDrifted =
            exitDebouncer.calculate(Math.abs(headingErrorRad) >= EXIT_HEADING_TOLERANCE_RAD);
        entryDebouncer.calculate(false);
        if (hasDriverInput || headingDrifted) {
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
