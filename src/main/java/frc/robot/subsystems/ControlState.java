package frc.robot.subsystems;

/**
 * Controls whether a subsystem is driven by the superstructure state machine or by direct commands.
 *
 * <p>When {@code SUPERSTRUCTURE}, the subsystem obeys state transitions from the superstructure.
 * When {@code INDEPENDENT}, superstructure state calls are ignored and commands drive hardware
 * directly.
 */
public enum ControlState {
  SUPERSTRUCTURE,
  INDEPENDENT
}
