// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private double rpmSetpoint = 0.0;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }
  public void setDutyCycke(double duty){
    io.setDutyCycle(duty);
  }
  public void setRPM(double rpm) {
    rpmSetpoint = rpm;
    io.setRPM(rpm, ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.stop();
  }

    public Command setDutyCycleCmd(double duty) {
    return this.runEnd(() -> io.setDutyCycle(duty), () -> io.setDutyCycle(0.0));
  }
  
}
