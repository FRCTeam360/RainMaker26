// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationConstants;

public class IndexerIOSim implements IndexerIO {
  /** Creates a new IndexerIOSim. */

  private DCMotor gearbox = DCMotor.getNeo550(1);
  private Encoder encoder = new Encoder(SimulationConstants.INDEXER_ENCODER1, SimulationConstants.INDEXER_ENCODER2);

  private final PWMSparkMax indexerMotor = new PWMSparkMax(SimulationConstants.INDEXER_MOTOR);
  public IndexerIOSim() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
