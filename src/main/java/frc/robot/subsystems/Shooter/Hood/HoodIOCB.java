package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class HoodIOCB extends HoodIOPB {
  private static final int HOOD_ID = CompBotConstants.HOOD_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;

  public HoodIOCB() {
    super(HOOD_ID, CANBUS);
  }
}
