package frc.robot.subsystems.Climber;

import frc.robot.Constants.CompBotConstants;

public class ClimberIOCB implements ClimberIO {
  private static final int CLIMBER_LEFT_ID = CompBotConstants.CLIMBER_LEFT_ID;
  private static final int CLIMBER_RIGHT_ID = CompBotConstants.CLIMBER_RIGHT_ID;
  private final ClimberIOPB io;

  public ClimberIOCB() {
    io = new ClimberIOPB(CLIMBER_LEFT_ID, CLIMBER_RIGHT_ID);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    io.updateInputs(inputs);
  }

  @Override
  public void setLeftDutyCycle(double dutyCycle) {
    io.setLeftDutyCycle(dutyCycle);
  }

  @Override
  public void setLeftPosition(double position) {
    io.setLeftPosition(position);
  }

  @Override
  public void setRightDutyCycle(double dutyCycle) {
    io.setRightDutyCycle(dutyCycle);
  }

  @Override
  public void setRightPosition(double position) {
    io.setRightPosition(position);
  }

  @Override
  public void zeroBoth() {
    io.zeroBoth();
  }
}
