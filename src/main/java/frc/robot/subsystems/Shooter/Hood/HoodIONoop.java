package frc.robot.subsystems.Shooter.Hood;

public class HoodIONoop implements HoodIO {
  @Override
  public void setZero() {}

  @Override
  public void setDutyCycle(double dutyCycle) {}

  @Override
  public void setPositionSmooth(double position) {}

  @Override
  public void setPositionAggressive(double position) {}
}
