// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum States{
    OFF,
    COLLECTING_FUEL
  }
   private States wantedState= States.OFF;

  private States currentState = States.OFF;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setDutyCycle(double value) {
    io.setDutyCycle(value);
  }

  public void stop() {
    this.setDutyCycle(0.0);
  }

  private States transition(){
     return switch (wantedState) {
      case COLLECTING_FUEL:
        break;
      default:
        yield States.OFF;
        break;
     }
  }

  private void applyState() {
        switch (currentState) {
          case COLLECTING_FUEL:

            break;
          case OFF:
          default:
            break;

        }
      }

  @Override
  public void periodic() {
    applyState();
    currentState = transition();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Subsystems/Intake/SystemState", currentState);
    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }
}
