package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum States {
    OFF,
    COLLECTING_FUEL
  }

  private States wantedState = States.OFF;
  private States currentState = States.OFF;
  private States previousState = States.OFF;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setWantedState(States state) {
    wantedState = state;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case COLLECTING_FUEL:
        currentState = States.COLLECTING_FUEL;
        break;

      case OFF:
      default:
        currentState = States.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case COLLECTING_FUEL:
        collectingFuel();
        break;
      case OFF:
      default:
        io.setDutyCycle(0.0);
        break;
    }
  }

  private void collectingFuel(){
    io.setDutyCycle(1.0);
  }

  @Override
  public void periodic() {
    updateState();
    applyState();

    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Intake/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Intake/PreviousState", previousState.toString());
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(
        () -> io.setDutyCycle(dutySupplier.getAsDouble()),
        () -> io.setDutyCycle(0.0)
    );
  }
}
