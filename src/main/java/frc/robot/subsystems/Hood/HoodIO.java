// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface HoodIO {
  /** Creates a new HoodIO. */
  @AutoLog
  public static class HoodIOInputs {
    public double hoodVoltage = 0.0;
    public double hoodSupplyCurrent = 0.0;
    public double hoodStatorCurrent = 0.0;
    public double hoodVelocity = 0.0;
    public double hoodPosition = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {
  }

  public void setDutyCycle(double dutyCycle);

  public void setPosition(double position);
}
