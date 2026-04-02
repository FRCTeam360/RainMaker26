package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class HoodIOCB implements HoodIO {
  private static final int HOOD_ID = CompBotConstants.HOOD_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;
  private final HoodIOPB io;

  public HoodIOCB() {
    io = new HoodIOPB(HOOD_ID, CANBUS);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    io.updateInputs(inputs);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  @Override
  public void setPosition(double position) {
    io.setPosition(position);
  }

  @Override
  public void setZero() {
    io.setZero();
  }
}
