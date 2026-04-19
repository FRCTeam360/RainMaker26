package frc.robot.subsystems.Shooter.Flywheel;

public class FlywheelIONoop implements FlywheelIO {
  @Override
  public void setSpinupVelocityControl(double velocityRPM) {}

  @Override
  public void setHoldVelocityControl(double velocityRPM) {}

  @Override
  public void setCoastVelocityControl(double velocityRPM) {}

  @Override
  public void setDutyCycle(double duty) {}
}
