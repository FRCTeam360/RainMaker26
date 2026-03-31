package frc.robot.subsystems.Climber;

import frc.robot.Constants.CompBotConstants;

public class ClimberIOCB extends ClimberIOPB {
  private static final int CLIMBER_LEFT_ID = CompBotConstants.CLIMBER_LEFT_ID;
  private static final int CLIMBER_RIGHT_ID = CompBotConstants.CLIMBER_RIGHT_ID;

  public ClimberIOCB() {
    super(CLIMBER_LEFT_ID, CLIMBER_RIGHT_ID);
  }
}
