// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import frc.robot.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;

/** Vision IO layer for Limelight 3. Sends robot orientation every frame for MegaTag2. */
public class VisionIOLimelight3G extends VisionIOLimelightBase {

  /**
   * Creates a new Limelight 3 hardware layer.
   *
   * @param name the NetworkTables name of the Limelight
   * @param gyroAngleSupplier supplies the robot's gyro angle in degrees
   * @param gyroAngleRateSupplier supplies the robot's gyro angular rate in degrees per second
   * @param acceptMeasurements whether to process pose estimates from this Limelight
   */
  public VisionIOLimelight3G(
      String name,
      DoubleSupplier gyroAngleSupplier,
      DoubleSupplier gyroAngleRateSupplier,
      boolean acceptMeasurements) {
    super(name, gyroAngleSupplier, gyroAngleRateSupplier, acceptMeasurements);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // LL3 needs robot orientation sent every frame for MegaTag2
    LimelightHelpers.SetRobotOrientation(
        getName(),
        gyroAngleSupplier.getAsDouble(),
        gyroAngleRateSupplier.getAsDouble(),
        0,
        0,
        0,
        0);
    super.updateInputs(inputs);
  }
}
