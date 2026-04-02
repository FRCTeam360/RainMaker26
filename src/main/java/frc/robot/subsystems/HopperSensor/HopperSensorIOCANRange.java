// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class HopperSensorIOCANRange implements HopperSensorIO {

  private final CANrange canRange;
  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> isDetectedSignal;

  /**
   * Creates a new HopperSensorIOCANRange.
   *
   * @param canId CAN ID of the CANRange sensor
   * @param canBus CAN bus name the sensor is on
   * @param config configuration to apply to the CANRange
   */
  public HopperSensorIOCANRange(int canId, String canBus, CANrangeConfiguration config) {
    canRange = new CANrange(canId, canBus);
    canRange.getConfigurator().apply(config);

    distanceSignal = canRange.getDistance();
    isDetectedSignal = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(50, distanceSignal, isDetectedSignal);
    canRange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HopperSensorIOInputs inputs) {
    BaseStatusSignal.refreshAll(distanceSignal, isDetectedSignal);
    inputs.distanceMeters = distanceSignal.getValueAsDouble();
    inputs.sensorActivated = isDetectedSignal.getValue();
  }
}
