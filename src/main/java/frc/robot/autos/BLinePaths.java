package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * All BLine path definitions for autonomous routines.
 *
 * <p>MASTER paths are defined once using Red Right coordinates. MIRRORED, FLIPPED, and FLIPPED
 * MIRRORED variants are derived by applying coordinate transformations.
 *
 * <p>This class is a pure data holder — no commands, no subsystem references.
 */
final class BLinePaths {

  private BLinePaths() {}

  static final double FIELD_LENGTH_METERS = 16.54;
  static final double FIELD_WIDTH_METERS = 8.07;

  // ─────────────────────────────────────────────────────────────────────────────
  // Coordinate transformations
  // ─────────────────────────────────────────────────────────────────────────────

  /** Mirrors Y coordinate about field centerline (Red Right → Red Left). */
  static Translation2d mirrorY(Translation2d t) {
    return new Translation2d(t.getX(), FIELD_WIDTH_METERS - t.getY());
  }

  /** Mirrors a rotation about the X axis (negates angle). */
  static Rotation2d mirrorRotation(Rotation2d r) {
    return r.unaryMinus();
  }

  /** Flips X coordinate about field centerline (Red → Blue). */
  static Translation2d flipX(Translation2d t) {
    return new Translation2d(FIELD_LENGTH_METERS - t.getX(), t.getY());
  }

  /** Reflects a rotation across the vertical axis (Red → Blue): π - rot. */
  static Rotation2d flipRotation(Rotation2d r) {
    return Rotation2d.k180deg.minus(r);
  }

  /** Applies both flip and mirror (Red Right → Blue Left). */
  static Translation2d flipAndMirror(Translation2d t) {
    return mirrorY(flipX(t));
  }

  static Rotation2d flipAndMirrorRotation(Rotation2d r) {
    return mirrorRotation(flipRotation(r));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Rotation constants
  // ─────────────────────────────────────────────────────────────────────────────

  private static final Rotation2d ROT_0 = Rotation2d.fromDegrees(0);
  private static final Rotation2d ROT_180 = Rotation2d.fromDegrees(180);
  private static final Rotation2d ROT_90 = Rotation2d.fromDegrees(90);
  private static final Rotation2d ROT_NEG_90 = Rotation2d.fromDegrees(-90);
}
