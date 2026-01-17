// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivotIO.IntakePivotIOInputs;

public class IntakePivot extends SubsystemBase {
  public final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  public final IntakePivotIO io;
  /** Creates a new IntakePivot. */
  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void setPosition(double value) {
    io.setPosition(value);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
