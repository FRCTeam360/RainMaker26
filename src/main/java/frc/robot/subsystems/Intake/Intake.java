package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum IntakeStates {
    OFF,
    COLLECTING_FUEL,
  }

  private IntakeStates wantedState = IntakeStates.OFF;
  private IntakeStates currentState = IntakeStates.OFF;
  private IntakeStates previousState = IntakeStates.OFF;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setWantedState(IntakeStates state) {
    wantedState = state;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case COLLECTING_FUEL:
        currentState = IntakeStates.COLLECTING_FUEL;
        break;

      case OFF:
      default:
        currentState = IntakeStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case COLLECTING_FUEL:
        setDutyCycle(-0.65);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }
  
  public void setDutyCycle(double duty){
    io.setDutyCycle(duty);
  }
  public void stop(){
    io.setDutyCycle(0.0);
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
