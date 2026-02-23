// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import frc.robot.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;

/**
 * Vision IO layer for Limelight 4. Adds IMU initialization and provides {@link #enableIMUSeeding()}
 * and {@link #enableIMUAssist()} overrides.
 */
public class VisionIOLimelight4 extends VisionIOLimelightBase {

  /** Uses only the external gyro (Pigeon) for IMU data, ignoring the LL4 internal IMU. */
  private static final int IMU_MODE_EXTERNAL = 2;

  /**
   * Creates a new Limelight 4 hardware layer. Sets IMU mode to use external gyro data only.
   *
   * @param name the NetworkTables name of the Limelight
   * @param gyroAngleSupplier supplies the robot's gyro angle in degrees
   * @param gyroAngleRateSupplier supplies the robot's gyro angular rate in degrees per second
   * @param acceptMeasurements whether to process pose estimates from this Limelight
   */
  public VisionIOLimelight4(
      String name,
      DoubleSupplier gyroAngleSupplier,
      DoubleSupplier gyroAngleRateSupplier,
      boolean acceptMeasurements) {
    super(name, gyroAngleSupplier, gyroAngleRateSupplier, acceptMeasurements);
    LimelightHelpers.SetIMUMode(name, IMU_MODE_EXTERNAL);
  }

  @Override
  public void enableIMUSeeding() {
    LimelightHelpers.SetIMUMode(getName(), IMU_MODE_EXTERNAL);
  }

  @Override
  public void enableIMUAssist() {
    LimelightHelpers.SetIMUMode(getName(), IMU_MODE_EXTERNAL);
  }

  @Override
  public void setThrottle(int throttle) {
    LimelightHelpers.SetThrottle(getName(), throttle);
  }
}
