// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HopperSensor extends SubsystemBase {

  private final HopperSensorIO io;
  private final HopperSensorIOInputsAutoLogged inputs = new HopperSensorIOInputsAutoLogged();

  public HopperSensor(HopperSensorIO io) {
    this.io = io;
  }

  /** Returns whether the sensor detects an object. */
  public boolean isActivated() {
    return inputs.sensorActivated;
  }

  /** Returns the distance reported by the sensor in meters. */
  public double getDistanceMeters() {
    return inputs.distanceMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("HopperSensor", inputs);
  }
}
