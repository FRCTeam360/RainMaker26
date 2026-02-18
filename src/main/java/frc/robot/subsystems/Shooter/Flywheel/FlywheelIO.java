package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  public static final int MAX_MOTORS = 2; // might become 3 might become 4

  @AutoLog
  public static class FlywheelIOInputs {
    public double[] statorCurrents = new double[MAX_MOTORS];
    public double[] supplyCurrents = new double[MAX_MOTORS];
    public double[] voltages = new double[MAX_MOTORS];
    public double[] velocities = new double[MAX_MOTORS];  // RPS (rotations per second)
    public double[] positions = new double[MAX_MOTORS];
  }

  /** Set flywheel velocity using duty cycle bang-bang control (fast acceleration) */
  public void setVelocityBangBang(double velocityRPS);

  /** Set flywheel velocity using torque current bang-bang control (consistent torque) */
  public void setVelocityTorqueCurrentBangBang(double velocityRPS);

  /** Set flywheel velocity using traditional PID control with torque current (Slot 2) */
  public void setVelocityPID(double velocityRPS);
  
  /** Set flywheel duty cycle directly (open loop) */
  public void setDutyCycle(double duty);

  /** Stop the flywheel */
  public void stop();
  public void setVelocity(double rpm);

  public default void updateInputs(FlywheelIOInputs inputs) {}
}
