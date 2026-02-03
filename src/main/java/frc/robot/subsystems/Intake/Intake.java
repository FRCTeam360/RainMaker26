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
    INTAKING
  }

  private IntakeStates wantedState = IntakeStates.OFF;
  private IntakeStates currentState = IntakeStates.OFF;
  private IntakeStates previousState = IntakeStates.OFF;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setWantedState(IntakeStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = IntakeStates.INTAKING;
        break;

      case OFF:
      default:
        currentState = IntakeStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        setVelocity(0.75);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public Command setVelocityCommand(double velocity) {
    return this.runEnd(() -> setVelocity(velocity), () -> setVelocity(0.0));
  }

  public void stop() {
    this.setVelocity(0.0);
  }

  @Override
  public void periodic() {

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
