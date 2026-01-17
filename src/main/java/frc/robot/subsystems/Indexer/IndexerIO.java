// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  /** Creates a new IndexerIO. */
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerVoltage = 0.0;
    public double indexerSupplyCurrent = 0.0;
    public double indexerStatorCurrent = 0.0;
    public double indexerVelocity = 0.0;
    public double indexerPosition = 0.0;
    public boolean indexerSensor = false;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);
}
