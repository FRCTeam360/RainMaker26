package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware IO interface for the flywheel subsystem. Implementations provide motor control and
 * sensor readings for different robot configurations (real hardware, simulation, noop).
 */
public interface FlywheelIO {
  /** Maximum number of flywheel motors supported. */
  public static final int MAX_MOTORS = 2;

  /** Logged inputs from the flywheel hardware. */
  @AutoLog
  public static class FlywheelIOInputs {
    public double[] statorCurrents = new double[MAX_MOTORS];
    public double[] supplyCurrents = new double[MAX_MOTORS];
    public double[] voltages = new double[MAX_MOTORS];
    public double[] velocities = new double[MAX_MOTORS];
    public double[] positions = new double[MAX_MOTORS];
  }

  /**
   * Sets flywheel velocity using duty-cycle bang-bang control for maximum acceleration. Used during
   * spinup and recovery phases.
   *
   * @param velocityRPM the target velocity in RPM
   */
  public void setSpinupVelocityControl(double velocityRPM);

  /**
   * Sets flywheel velocity using torque-current bang-bang control for consistent, bounded torque.
   * Used when holding at setpoint.
   *
   * @param velocityRPM the target velocity in RPM
   */
  public void setHoldVelocityControl(double velocityRPM);

  /**
   * Sets flywheel velocity using traditional PID control for smooth operation.
   *
   * @param velocityRPM the target velocity in RPM
   */
  public void setCoastVelocityControl(double velocityRPM);

  public void setDutyCycle(double duty);

  public default void updateInputs(FlywheelIOInputs inputs) {}
}
