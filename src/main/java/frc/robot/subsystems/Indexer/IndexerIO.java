// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  /** Creates a new IndexerIO. */
  @AutoLog
  public static class IndexerIOInputs {
    public double voltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    // capacity sensor
    public boolean sensor = false;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);
}
