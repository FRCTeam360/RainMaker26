package frc.robot.subsystems.Intake.IntakePivot;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class IntakePivotIOCB extends IntakePivotIOPB {
  private static final int INTAKE_PIVOT_ID = CompBotConstants.INTAKE_PIVOT_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;

  public IntakePivotIOCB() {
    super(INTAKE_PIVOT_ID, CANBUS);
  }
}
