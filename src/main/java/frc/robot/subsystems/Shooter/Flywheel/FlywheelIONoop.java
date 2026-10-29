package frc.robot.subsystems.Shooter.Flywheel;

public class FlywheelIONoop implements FlywheelIO {
  public void setSpinupVelocityControl(double velocityRPM) {}

  public void setHoldVelocityControl(double velocityRPM) {}

  public void setCoastVelocityControl(double velocityRPM) {}

  public void setDutyCycle(double duty) {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < inputs.connected.length; i++) {
      inputs.connected[i] = true;
    }
  }
}
