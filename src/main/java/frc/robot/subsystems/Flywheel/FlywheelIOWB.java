// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelIOWB implements FlywheelIO {

  // need motor vvvvvv
  private final SparkMax motor = new SparkMax(0, null);

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }
}
