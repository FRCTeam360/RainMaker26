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
   *   <li>{@code LIVE} — internal state mirrors the sensor directly every cycle.
   *   <li>{@code LATCHED} — internal state is frozen once FULL; only resets to NOT_FULL on a
   *       falling edge (sensor transitions from activated to not activated).
   * </ul>
   */
  public enum HopperSensorWantedStates {
    LIVE,
    LATCHED
  }

  /**
   * Internal states representing whether balls are currently stacked over the spindexer.
   *
   * <ul>
   *   <li>{@code NOT_FULL} — no balls detected over the spindexer; agitate at high intensity.
   *   <li>{@code FULL} — balls detected over the spindexer; back off to low intensity.
   * </ul>
   */
  public enum HopperSensorInternalStates {
    NOT_FULL,
    FULL
  }

  // State variables
  private HopperSensorWantedStates wantedState = HopperSensorWantedStates.LIVE;
  private HopperSensorInternalStates currentState = HopperSensorInternalStates.NOT_FULL;
  private HopperSensorInternalStates previousState = HopperSensorInternalStates.NOT_FULL;

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

  /** Returns the distance reported by the sensor in meters. */
  public double getDistanceMeters() {
    return inputs.distanceMeters;
  }

  /**
   * Updates the sensor state based on the current wanted state and sensor readings. Called every
   * cycle from periodic().
   */
  private void updateState() {
    previousState = currentState;

    boolean previousDebouncedSensorActivated = debouncedSensorActivated;
    debouncedSensorActivated = sensorActivatedDebouncer.calculate(inputs.sensorActivated);

    switch (wantedState) {
      case LIVE:
        // Update freely from the sensor every cycle.
        currentState =
            debouncedSensorActivated
                ? HopperSensorInternalStates.FULL
                : HopperSensorInternalStates.NOT_FULL;
        break;
      case LATCHED:
        // Latch frozen — only reset to NOT_FULL on a falling edge (balls have cleared).
        if (previousState == HopperSensorInternalStates.FULL && !debouncedSensorActivated) {
          currentState = HopperSensorInternalStates.NOT_FULL;
        }
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("HopperSensor", inputs);

    updateState();

    Logger.recordOutput("Subsystems/HopperSensor/WantedState", wantedState);
    Logger.recordOutput("Subsystems/HopperSensor/CurrentState", currentState);
    Logger.recordOutput("Subsystems/HopperSensor/PreviousState", previousState);
  }
}
