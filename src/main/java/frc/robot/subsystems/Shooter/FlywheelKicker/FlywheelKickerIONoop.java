package frc.robot.subsystems.Shooter.FlywheelKicker;

public class FlywheelKickerIONoop implements FlywheelKickerIO {
  @Override
  public void setDutyCycle(double dutyCycle) {}

  @Override
  public void setVelocity(double rpm) {}

  @Override
  public void setSpinupVelocityControl(double rpm) {}

  @Override
  public void setHoldVelocityControl(double rpm) {}
}
