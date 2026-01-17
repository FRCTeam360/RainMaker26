// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.ControlType;

public interface FlywheelIO {
  public static final int MAX_MOTORS = 2;//might become 3 might become 4
  @AutoLog
  public static class FlywheelIOInputs{
    public double[] StatorCurrents = new double[MAX_MOTORS];
    public double flywheelVoltages = 0.0;
    public double flywheelVelocitys = 0.0;
    public double flywheelPositions = 0.0;
  }
  public void setDutyCycle(double duty);
  public void setSpeed(double speed);
  public void setRPM(double rpm, ControlType kvelocity);
  public void stop();
  public double getPower();
  public double getVelocity();
  public default void updateInputs(FlywheelIOInputs inputs) {}

}
