package frc.robot.subsystems.Intake.IntakeRoller;

import frc.robot.Constants.CompBotConstants;

public class IntakeRollerIOCB extends IntakeRollerIOPB {
  private static final int LEFT_INTAKE_ROLLER_ID = CompBotConstants.LEFT_INTAKE_ROLLER_ID;
  private static final int RIGHT_INTAKE_ROLLER_ID = CompBotConstants.RIGHT_INTAKE_ROLLER_ID;

  public IntakeRollerIOCB() {
    super(LEFT_INTAKE_ROLLER_ID, RIGHT_INTAKE_ROLLER_ID);
  }
}
