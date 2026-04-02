package frc.robot.subsystems.Indexer;

import frc.robot.Constants.CompBotConstants;

public class IndexerIOCB implements IndexerIO {
  private static final int TWINDEXER_ID = CompBotConstants.TWINDEXER_ID;
  private final IndexerIOPB io;

  public IndexerIOCB() {
    io = new IndexerIOPB(TWINDEXER_ID);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    io.updateInputs(inputs);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  @Override
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }
}
