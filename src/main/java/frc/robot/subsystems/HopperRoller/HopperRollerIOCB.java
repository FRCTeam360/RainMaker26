package frc.robot.subsystems.HopperRoller;

import frc.robot.Constants.CompBotConstants;

public class HopperRollerIOCB implements HopperRollerIO {
  private static final int HOPPER_ROLLER_ID = CompBotConstants.HOPPER_ROLLER_ID;
  private final HopperRollerIOPB io;

  public HopperRollerIOCB() {
    io = new HopperRollerIOPB(HOPPER_ROLLER_ID);
  }

  @Override
  public void updateInputs(HopperRollerIOInputs inputs) {
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
}
