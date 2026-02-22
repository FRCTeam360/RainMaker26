// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;

/**
 * Vision IO layer for Limelight 4. Adds IMU initialization and provides {@link #seedIMU()} and
 * {@link #enableIMUAssist()} overrides.
 */
public class VisionIOLimelight4 extends VisionIOLimelightBase {

  /**
   * Creates a new Limelight 4 hardware layer. Configures IMU assist alpha and sets initial IMU mode
   * to external seed.
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
    LimelightHelpers.SetIMUAssistAlpha(name, Constants.IMU_ASSIST_ALPHA);
    LimelightHelpers.SetIMUMode(name, Constants.IMU_MODE_EXTERNAL_SEED);
  }

  @Override
  public void seedIMU() {
    LimelightHelpers.SetIMUMode(getName(), Constants.IMU_MODE_EXTERNAL_SEED);
  }

  @Override
  public void enableIMUAssist() {
    LimelightHelpers.SetIMUMode(getName(), Constants.IMU_MODE_INTERNAL_EXTERNAL_ASSIST);
  }

  @Override
  public void setThrottle(int throttle) {
    LimelightHelpers.SetThrottle(getName(), throttle);
  }
}
