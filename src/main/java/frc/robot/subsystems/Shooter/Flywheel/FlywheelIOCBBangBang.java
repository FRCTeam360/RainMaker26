package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class FlywheelIOCBBangBang implements FlywheelIO {
  private static final int FLYWHEEL_RIGHT_ID = CompBotConstants.FLYWHEEL_RIGHT_ID;
  private static final int FLYWHEEL_LEFT_ID = CompBotConstants.FLYWHEEL_LEFT_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;
  private final FlywheelIOPBBangBang io;

  public FlywheelIOCBBangBang() {
    io = new FlywheelIOPBBangBang(FLYWHEEL_RIGHT_ID, FLYWHEEL_LEFT_ID, CANBUS);
  }

  @Override
  public void setSpinupVelocityControl(double velocityRPM) {
    io.setSpinupVelocityControl(velocityRPM);
  }

  @Override
  public void setHoldVelocityControl(double velocityRPM) {
    io.setHoldVelocityControl(velocityRPM);
  }

  @Override
  public void setCoastVelocityControl(double velocityRPM) {
    io.setCoastVelocityControl(velocityRPM);
  }

  @Override
  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    io.updateInputs(inputs);
  }
}
