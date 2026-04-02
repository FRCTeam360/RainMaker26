package frc.robot.subsystems.Intake.IntakePivot;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CompBotConstants;

public class IntakePivotIOCB implements IntakePivotIO {
  private static final int INTAKE_PIVOT_ID = CompBotConstants.INTAKE_PIVOT_ID;
  private static final CANBus CANBUS = CompBotConstants.CANBUS;
  private final IntakePivotIOPB io;

  public IntakePivotIOCB() {
    io = new IntakePivotIOPB(INTAKE_PIVOT_ID, CANBUS);
  }

  @Override
  public void setZero() {
    io.setZero();
  }

  @Override
  public void setPosition(double position) {
    io.setPosition(position);
  }

  @Override
  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    io.updateInputs(inputs);
  }

  @Override
  public void enableBrakeMode() {
    io.enableBrakeMode();
  }

  @Override
  public void disableBrakeMode() {
    io.disableBrakeMode();
  }
}
