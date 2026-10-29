package frc.robot.subsystems.Shooter.Hood;

public class HoodIONoop implements HoodIO {
  public void setZero() {}

  public void setDutyCycle(double dutyCycle) {}

  public void setPositionSmooth(double position) {}

  public void setPositionAggressive(double position) {}

  @Override
  public void updateInputs(HoodIOInputs inputs){
    inputs.connected = true;
  }
}
