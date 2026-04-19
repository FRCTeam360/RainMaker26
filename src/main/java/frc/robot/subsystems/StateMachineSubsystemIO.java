package frc.robot.subsystems;

public interface StateMachineSubsystemIO<I> {

  public default void updateInputs(I inputs) {}
}
