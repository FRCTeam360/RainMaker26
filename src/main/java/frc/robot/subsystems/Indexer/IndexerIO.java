// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import frc.robot.subsystems.StateMachineSubsystemIO;

public interface IndexerIO extends StateMachineSubsystemIO<IndexerIOInputsAutoLogged> {
  /** Creates a new IndexerIO. */
  public void setDutyCycle(double dutyCycle);

  public void setVelocity(double velocity);
}
