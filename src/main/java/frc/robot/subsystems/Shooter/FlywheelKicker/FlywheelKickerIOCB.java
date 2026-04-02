package frc.robot.subsystems.Shooter.FlywheelKicker;

import frc.robot.Constants.CompBotConstants;

public class FlywheelKickerIOCB implements FlywheelKickerIO {
  private static final int FLYWHEEL_KICKER_ID = CompBotConstants.FLYWHEEL_KICKER_ID;
  private final FlywheelKickerIOPB io;

  public FlywheelKickerIOCB() {
    io = new FlywheelKickerIOPB(FLYWHEEL_KICKER_ID);
  }

  @Override
  public void updateInputs(FlywheelKickerIOInputs inputs) {
    io.updateInputs(inputs);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  @Override
  public void setVelocity(double rpm) {
    io.setVelocity(rpm);
  }

  @Override
  public void setSpinupVelocityControl(double rpm) {
    io.setSpinupVelocityControl(rpm);
  }

  @Override
  public void setHoldVelocityControl(double rpm) {
    io.setHoldVelocityControl(rpm);
  }
}
