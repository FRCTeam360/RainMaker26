package frc.robot.subsystems.Shooter.Hood;

public class HoodIONoop implements HoodIO {
  public boolean connected = true;

  public void setZero() {}

  public void setDutyCycle(double dutyCycle) {}

  public void setPositionSmooth(double position) {}

  public void setPositionAggressive(double position) {}
}
