// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated hopper sensor IO. Toggles between activated and not activated every {@link
 * #TOGGLE_PERIOD_SECONDS} seconds to exercise the agitation state machine.
 */
public class HopperSensorIOSim implements HopperSensorIO {
  private static final double TOGGLE_PERIOD_SECONDS = 2.0;

  private final Timer timer = new Timer();

  public HopperSensorIOSim() {
    timer.start();
  }

  @Override
  public void updateInputs(HopperSensorIOInputs inputs) {
    double elapsed = timer.get();
    // Activated for the first half of each period, deactivated for the second half
    inputs.sensorActivated = (elapsed % TOGGLE_PERIOD_SECONDS) < (TOGGLE_PERIOD_SECONDS / 2.0);
    inputs.distanceMeters = inputs.sensorActivated ? 0.05 : 2.0;
    inputs.connected = true;
  }
}
