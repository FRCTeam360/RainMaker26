// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import org.littletonrobotics.junction.AutoLog;

public interface HopperSensorIO {

  @AutoLog
  public static class HopperSensorIOInputs {
    public boolean sensorActivated = false;
    public double distanceMeters = 0.0;
  }

  public default void updateInputs(HopperSensorIOInputs inputs) {}
}
