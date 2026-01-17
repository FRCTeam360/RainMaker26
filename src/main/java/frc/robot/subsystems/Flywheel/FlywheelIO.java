// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  public static final int MAX_MOTORS = 2;// might become 3 might become 4

  @AutoLog
  public static class FlywheelIOInputs {
    public double[] statorCurrents = new double[MAX_MOTORS];
    public double[] voltages = new double[MAX_MOTORS];
    public double[] velocities = new double[MAX_MOTORS];
    public double[] positions = new double[MAX_MOTORS];
  }

  public void setDutyCycle(double duty);

  public void setRPM(double rpm);

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

}
