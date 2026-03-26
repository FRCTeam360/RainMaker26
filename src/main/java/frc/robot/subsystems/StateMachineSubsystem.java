package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class StateMachineSubsystem<
        I extends LoggableInputs, IO extends StateMachineSubsystemIO<I>>
    extends SubsystemBase {

  protected final IO io;
  protected final I inputs;

  protected ControlState controlState = ControlState.SUPERSTRUCTURE;

  public final String logKey;

  protected StateMachineSubsystem(IO io, I inputs) {
    this.io = io;
    this.inputs = inputs;
    logKey = this.getClass().getSimpleName();
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
  }

  protected abstract void logOutputs();

  protected abstract void updateState();

  protected abstract void applyState();
}
