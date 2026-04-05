// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperSensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;

public class HopperSensorIOCANRange implements HopperSensorIO {

  private static final int SENSOR_UPDATE_FREQUENCY_HZ = 50;
  private static final int MIN_SIGNAL_STRENGTH = 2000;
  private static final double PROXIMITY_THRESHOLD_METERS = 0.22;
  private static final double PROXIMITY_HYSTERESIS_METERS = 0.01;
  private static final double FOV_CENTER_X_DEGREES = 0.0;
  private static final double FOV_CENTER_Y_DEGREES = 0.0;
  private static final double FOV_RANGE_X_DEGREES = 27.0;
  private static final double FOV_RANGE_Y_DEGREES = 27.0;

  private final CANrange canRange;

  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> isDetectedSignal;

  public HopperSensorIOCANRange(int sensorId, CANBus canBus) {
    canRange = new CANrange(sensorId, canBus);
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH;
    config.ProximityParams.ProximityThreshold = PROXIMITY_THRESHOLD_METERS;
    config.ProximityParams.ProximityHysteresis = PROXIMITY_HYSTERESIS_METERS;
    config.FovParams.FOVCenterX = FOV_CENTER_X_DEGREES;
    config.FovParams.FOVCenterY = FOV_CENTER_Y_DEGREES;
    config.FovParams.FOVRangeX = FOV_RANGE_X_DEGREES;
    config.FovParams.FOVRangeY = FOV_RANGE_Y_DEGREES;
    config.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);
    canRange.getConfigurator().apply(config);

    distanceSignal = canRange.getDistance();
    isDetectedSignal = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        SENSOR_UPDATE_FREQUENCY_HZ, distanceSignal, isDetectedSignal);
    canRange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HopperSensorIOInputs inputs) {
    BaseStatusSignal.refreshAll(distanceSignal, isDetectedSignal);
    inputs.distanceMeters = distanceSignal.getValueAsDouble();
    inputs.sensorActivated = isDetectedSignal.getValue();
    inputs.connected = distanceSignal.getStatus().isOK() && isDetectedSignal.getStatus().isOK();
  }
}
