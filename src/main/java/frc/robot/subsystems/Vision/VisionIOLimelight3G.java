// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.DoubleSupplier;

/** Vision IO layer for Limelight 3G. Inherits robot orientation handling from the base class. */
public class VisionIOLimelight3G extends VisionIOLimelightBase {

  /**
   * Creates a new Limelight 3G hardware layer.
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

  /**
   * Creates a new Limelight 3G hardware layer with a custom standard deviation map.
   *
   * @param name the NetworkTables name of the Limelight
   * @param gyroAngleSupplier supplies the robot's gyro angle in degrees
   * @param gyroAngleRateSupplier supplies the robot's gyro angular rate in degrees per second
   * @param acceptMeasurements whether to process pose estimates from this Limelight
   * @param customStdDevMap distance-to-std-dev map for this camera
   */
  public VisionIOLimelight3G(
      String name,
      DoubleSupplier gyroAngleSupplier,
      DoubleSupplier gyroAngleRateSupplier,
      boolean acceptMeasurements,
      InterpolatingMatrixTreeMap<Double, N3, N1> customStdDevMap) {
    super(name, gyroAngleSupplier, gyroAngleRateSupplier, acceptMeasurements, customStdDevMap);
  }
}
