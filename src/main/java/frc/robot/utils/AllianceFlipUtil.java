package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for flipping field coordinates based on the current driver station alliance. When
 * on the Red alliance, coordinates are mirrored across the field so that autonomous and targeting
 * logic written for the Blue alliance side works correctly from either side.
 */
public class AllianceFlipUtil {

  /**
   * Flips an X coordinate across the field length if on the Red alliance.
   *
   * @param x the X coordinate in meters
   * @return the flipped X coordinate, or the original if on the Blue alliance
   */
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  /**
   * Flips a Y coordinate across the field width if on the Red alliance.
   *
   * @param y the Y coordinate in meters
   * @return the flipped Y coordinate, or the original if on the Blue alliance
   */
  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  /**
   * Flips a {@link Translation2d} across the field if on the Red alliance.
   *
   * @param translation the translation to flip
   * @return the flipped translation, or the original if on the Blue alliance
   */
  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  /**
   * Rotates a {@link Rotation2d} by 180 degrees if on the Red alliance.
   *
   * @param rotation the rotation to flip
   * @return the flipped rotation, or the original if on the Blue alliance
   */
  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  /**
   * Flips a {@link Pose2d} across the field if on the Red alliance.
   *
   * @param pose the pose to flip
   * @return the flipped pose, or the original if on the Blue alliance
   */
  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  /**
   * Flips a {@link Translation3d} across the field if on the Red alliance. The Z component is
   * preserved.
   *
   * @param translation the 3D translation to flip
   * @return the flipped translation, or the original if on the Blue alliance
   */
  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  /**
   * Rotates a {@link Rotation3d} by 180 degrees around the Z axis if on the Red alliance.
   *
   * @param rotation the 3D rotation to flip
   * @return the flipped rotation, or the original if on the Blue alliance
   */
  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  /**
   * Flips a {@link Pose3d} across the field if on the Red alliance.
   *
   * @param pose the 3D pose to flip
   * @return the flipped pose, or the original if on the Blue alliance
   */
  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  /**
   * Returns whether field coordinates should be flipped (i.e., the robot is on the Red alliance).
   *
   * @return {@code true} if on the Red alliance, {@code false} otherwise
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
