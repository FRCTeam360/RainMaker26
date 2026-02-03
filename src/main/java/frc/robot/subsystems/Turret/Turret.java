package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
// note: I believe this is right. For "Implement Turret Subsystem"
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double TOLERANCE = 0.5; // TODO: find actual one

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    // PID
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setEncoder(double position) {
    io.setEncoder(position);
  }

  public void stop() {
    io.setDutyCycle(0);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(inputs.position - setpoint) < TOLERANCE;
  }

  public Command setPositionCommand(double position) {
    return this.setPositionCommand(() -> position);
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return this.runEnd(() -> setPosition(position.getAsDouble()), () -> this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }
}
