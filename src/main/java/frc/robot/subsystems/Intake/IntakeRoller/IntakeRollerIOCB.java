package frc.robot.subsystems.Intake.IntakeRoller;

import frc.robot.Constants.CompBotConstants;

public class IntakeRollerIOCB implements IntakeRollerIO {
  private static final int LEFT_INTAKE_ROLLER_ID = CompBotConstants.LEFT_INTAKE_ROLLER_ID;
  private static final int RIGHT_INTAKE_ROLLER_ID = CompBotConstants.RIGHT_INTAKE_ROLLER_ID;
  private final IntakeRollerIOPB io;

  public IntakeRollerIOCB() {
    io = new IntakeRollerIOPB(LEFT_INTAKE_ROLLER_ID, RIGHT_INTAKE_ROLLER_ID);
  }

  @Override
  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  @Override
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  @Override
  public void stop() {
    io.stop();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    io.setPID(kP, kI, kD, kV, kS);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    io.updateInputs(inputs);
  }
}
