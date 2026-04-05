// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HopperSensor extends SubsystemBase {

  private static final double SENSOR_ACTIVATED_DEBOUNCE_SECONDS = 0.2;

  // IO fields
  private final HopperSensorIO io;
  private final HopperSensorIOInputsAutoLogged inputs = new HopperSensorIOInputsAutoLogged();
  private final Debouncer sensorActivatedDebouncer =
      new Debouncer(SENSOR_ACTIVATED_DEBOUNCE_SECONDS, DebounceType.kBoth);

  private boolean debouncedSensorActivated = false;

  // Enums

  /**
   * Wanted states for the hopper sensor, controlling how the internal state is updated.
   *
   * <ul>
   *   <li>{@code NOT_AGITATING} — internal state mirrors the sensor directly every cycle (live
   *       tracking).
   *   <li>{@code AGITATING} — internal state latches FULL and only resets to HALF_EMPTY on a
   *       falling edge (sensor transitions from activated to not activated), providing hysteresis
   *       during agitation.
   * </ul>
   */
  public enum HopperSensorWantedStates {
    NOT_AGITATING,
    AGITATING
  }

  /**
   * Internal states representing whether balls are currently stacked over the spindexer.
   *
   * <ul>
   *   <li>{@code HALF_EMPTY} — no balls detected over the spindexer; agitate at high intensity.
   *   <li>{@code FULL} — balls detected over the spindexer; back off to low intensity.
   * </ul>
   */
  public enum HopperSensorInternalStates {
    HALF_EMPTY,
    FULL
  }

  // State variables
  private HopperSensorWantedStates wantedState = HopperSensorWantedStates.NOT_AGITATING;
  private HopperSensorInternalStates currentState = HopperSensorInternalStates.HALF_EMPTY;
  private HopperSensorInternalStates previousState = HopperSensorInternalStates.HALF_EMPTY;

  public HopperSensor(HopperSensorIO io) {
    this.io = io;
  }

  // State machine API

  /**
   * Sets the wanted state of the hopper sensor.
   *
   * @param state the desired hopper sensor state
   */
  public void setWantedState(HopperSensorWantedStates state) {
    wantedState = state;
  }

  /** Returns the current internal state of the hopper sensor. */
  public HopperSensorInternalStates getState() {
    return currentState;
  }

  /**
   * Updates the sensor state based on the current wanted state and sensor readings. Called every
   * cycle from periodic().
   */
  private void updateState() {
    previousState = currentState;

    if (!inputs.connected) {
      currentState = HopperSensorInternalStates.FULL;
      return;
    }

    debouncedSensorActivated = sensorActivatedDebouncer.calculate(inputs.sensorActivated);

    switch (wantedState) {
      case NOT_AGITATING:
        // Mirror sensor directly every cycle.
        currentState =
            debouncedSensorActivated
                ? HopperSensorInternalStates.FULL
                : HopperSensorInternalStates.HALF_EMPTY;
        break;
      case AGITATING:
        // One-way latch — only reset FULL to HALF_EMPTY on a falling edge (balls have cleared).
        // If we entered agitation while HALF_EMPTY, stay HALF_EMPTY (agitate at high intensity).
        if (previousState == HopperSensorInternalStates.FULL && !debouncedSensorActivated) {
          currentState = HopperSensorInternalStates.HALF_EMPTY;
        }
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/HopperSensor", inputs);

    updateState();

    Logger.recordOutput("Superstructure/Subsystems/HopperSensor/WantedState", wantedState);
    Logger.recordOutput("Superstructure/Subsystems/HopperSensor/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Subsystems/HopperSensor/PreviousState", previousState);
    Logger.recordOutput("Superstructure/Subsystems/HopperSensor/Connected", inputs.connected);
  }
}
