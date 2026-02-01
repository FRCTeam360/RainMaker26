// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

public class FlywheelIOSim implements FlywheelIO {
  /** Creates a new FlywheelIOSim. */
  public FlywheelIOSim() {}

  @Override
  public void setDutyCycle(double duty) {
    // TODO Auto-generated method stub
   System.out.println("AHHHHH but flywheel");
  }

  @Override
  public void setRPM(double rpm) {
    // TODO Auto-generated method stub
     System.out.println("AHHHHH but flywheel");
  }
}
