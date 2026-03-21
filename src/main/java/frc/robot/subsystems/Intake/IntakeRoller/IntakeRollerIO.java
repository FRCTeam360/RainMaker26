// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRoller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  /** Creates a new IntakeRollerIO. */
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double[] statorCurrent = new double[2];
    public double supplyCurrent = 0.0;
    public double[] voltage = new double[2];
    public double[] velocity = new double[2];
    public double[] position = new double[2];
    public boolean sensor = false;
    // insert inputs
  }

  public void setDutyCycle(double value);

  public void setVelocity(double velocity);

  public void stop();

  public default void updateInputs(IntakeRollerIOInputs inputs) {}
}
