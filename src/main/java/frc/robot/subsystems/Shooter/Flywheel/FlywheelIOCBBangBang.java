package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class FlywheelIOCBBangBang extends FlywheelIOPBBangBang {
  private static final int FLYWHEEL_RIGHT_ID = CompBotConstants.FLYWHEEL_RIGHT_ID;
  private static final int FLYWHEEL_LEFT_ID = CompBotConstants.FLYWHEEL_LEFT_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;

  public FlywheelIOCBBangBang() {
    super(FLYWHEEL_RIGHT_ID, FLYWHEEL_LEFT_ID, CANBUS);
  }
}
