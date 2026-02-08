// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  public final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  public final IntakePivotIO io;
  private final IntakePivotVisualizer visualizer;

  /** Creates a new IntakePivot. */
  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    // Initialize visualizer with arm length in meters (30 inches = 0.762 m)
    this.visualizer = new IntakePivotVisualizer(0.762);
  }

  public void setPosition(double value) {
    io.setPosition(value);
  }

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }

  public Command setPositionCommand(DoubleSupplier positionSupplier) {
    return this.runEnd(
        () -> this.setPosition(positionSupplier.getAsDouble()), () -> this.setPosition(90.0));
  }

  public Command setPositionCommand(double position) {
    return this.setPositionCommand(() -> position);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Update visualization with current arm angle (convert rotations to radians)
    visualizer.update(inputs.position * 2.0 * Math.PI);
  }
}
