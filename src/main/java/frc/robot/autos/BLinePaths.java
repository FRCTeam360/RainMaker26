package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

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

  /** Flips a rotation by 180 degrees (Red → Blue). */
  static Rotation2d flipRotation(Rotation2d r) {
    return r.rotateBy(Rotation2d.k180deg);
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

  // ─────────────────────────────────────────────────────────────────────────────
  // MASTER (Red Right) paths
  // ─────────────────────────────────────────────────────────────────────────────

  // ── No Stop Right Red Middle paths ──

  static Path noStopRightRedMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 7.485), ROT_180),
        new TranslationTarget(9.089, 7.290),
        new RotationTarget(Rotation2d.fromDegrees(-117.273), 0.5),
        new TranslationTarget(9.089, 5.302),
        new RotationTarget(Rotation2d.fromDegrees(-78.434), 0.5),
        new TranslationTarget(10.638, 5.536),
        new RotationTarget(Rotation2d.fromDegrees(-9.644), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)));
  }

  static Path noStopRightRedMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)),
        new TranslationTarget(14.623, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-173.062), 0.5),
        new TranslationTarget(13.444, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-169.236), 0.5),
        new TranslationTarget(10.385, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-85.796), 0.5),
        new TranslationTarget(10.570, 5.107),
        new RotationTarget(Rotation2d.fromDegrees(-87.075), 0.5),
        new TranslationTarget(9.995, 4.396),
        new RotationTarget(Rotation2d.fromDegrees(164.633), 0.5),
        new TranslationTarget(8.855, 5.273),
        new RotationTarget(Rotation2d.fromDegrees(10.025), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(21.501)));
  }

  // ── Right Red Middle Hawk paths ──

  static Path rightRedMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 7.485), ROT_180),
        new TranslationTarget(8.670, 7.358),
        new RotationTarget(Rotation2d.fromDegrees(-95.919), 0.5),
        new TranslationTarget(8.670, 4.464),
        new RotationTarget(Rotation2d.fromDegrees(-64.529), 0.5),
        new TranslationTarget(10.404, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-24.079), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)));
  }

  static Path rightRedMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)),
        new TranslationTarget(14.623, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-173.062), 0.5),
        new TranslationTarget(13.444, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-172.367), 0.5),
        new TranslationTarget(10.531, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-79.167), 0.5),
        new TranslationTarget(10.726, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-102.987), 0.5),
        new TranslationTarget(8.572, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-179.864), 0.5),
        new TranslationTarget(8.767, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-17.886), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(21.501)));
  }

  // ── Standard Swipe paths (Hook auto) ──

  static Path standardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(12.096, 7.436), Rotation2d.fromDegrees(179.384)),
        new TranslationTarget(8.774, 5.468),
        new RotationTarget(ROT_NEG_90, 0.5),
        new TranslationTarget(10.301, 5.279),
        new RotationTarget(Rotation2d.fromDegrees(40.0), 0.5),
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(40.764)));
  }

  static Path standardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(40.764)),
        new TranslationTarget(13.104, 7.373),
        new RotationTarget(ROT_180, 0.5),
        new TranslationTarget(10.774, 7.373),
        new RotationTarget(Rotation2d.fromDegrees(-179.148), 0.5),
        new TranslationTarget(10.569, 3.925),
        new RotationTarget(Rotation2d.fromDegrees(-69.738), 0.5),
        new TranslationTarget(8.947, 3.657),
        new RotationTarget(Rotation2d.fromDegrees(145.0), 0.5),
        new TranslationTarget(9.120, 5.373),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(45.0)));
  }

  // ── Trench Middle paths (Red middle 1-7) ──

  static Path redMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.090, 7.505), ROT_180),
        new Waypoint(new Translation2d(8.514, 7.202), Rotation2d.fromDegrees(-79.16)));
  }

  static Path redMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(8.514, 7.202), Rotation2d.fromDegrees(-79.16)),
        new Waypoint(new Translation2d(8.514, 5.058), Rotation2d.fromDegrees(-79.16)));
  }

  static Path redMiddle3() {
    return new Path(
        new Waypoint(new Translation2d(8.514, 5.058), Rotation2d.fromDegrees(-79.16)),
        new TranslationTarget(10.094, 7.600),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(13.337, 7.348),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.678, 5.604), Rotation2d.fromDegrees(52.058)));
  }

  static Path redMiddle5() {
    return new Path(
        new Waypoint(new Translation2d(13.678, 5.604), Rotation2d.fromDegrees(52.058)),
        new TranslationTarget(12.889, 7.465),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(10.275, 7.465),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(10.043, 7.046), Rotation2d.fromDegrees(-75.619)));
  }

  static Path redMiddle6() {
    return new Path(
        new Waypoint(new Translation2d(10.043, 7.046), Rotation2d.fromDegrees(-75.619)),
        new Waypoint(new Translation2d(10.780, 5.258), ROT_NEG_90));
  }

  static Path redMiddle7() {
    return new Path(
        new Waypoint(new Translation2d(10.780, 5.258), ROT_NEG_90),
        new TranslationTarget(9.790, 7.494),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(12.411, 7.494),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.610, 6.715), Rotation2d.fromDegrees(57.724)));
  }

  // ── Depot paths ──

  static Path depot1() {
    return new Path(
        new Waypoint(new Translation2d(12.996, 1.969), ROT_NEG_90),
        new Waypoint(new Translation2d(14.185, 1.784), Rotation2d.fromDegrees(-45.785)));
  }

  static Path depot2() {
    return new Path(
        new Waypoint(new Translation2d(14.185, 1.784), Rotation2d.fromDegrees(-45.785)),
        new TranslationTarget(16.056, 0.585),
        new RotationTarget(ROT_90, 0.5),
        new Waypoint(new Translation2d(16.046, 3.129), ROT_90));
  }

  static Path depot3() {
    return new Path(
        new Waypoint(new Translation2d(16.046, 3.129), ROT_90),
        new Waypoint(new Translation2d(13.824, 3.129), Rotation2d.fromDegrees(-45.785)));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // MIRRORED (Red Left) paths
  // ─────────────────────────────────────────────────────────────────────────────

  static Path mirroredNoStopLeftRedMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 0.585), ROT_180),
        new TranslationTarget(9.089, 0.780),
        new RotationTarget(Rotation2d.fromDegrees(117.273), 0.5),
        new TranslationTarget(9.089, 2.768),
        new RotationTarget(Rotation2d.fromDegrees(78.434), 0.5),
        new TranslationTarget(10.638, 2.534),
        new RotationTarget(Rotation2d.fromDegrees(9.644), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)));
  }

  static Path mirroredNoStopLeftRedMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)),
        new TranslationTarget(14.623, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(173.062), 0.5),
        new TranslationTarget(13.444, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(169.236), 0.5),
        new TranslationTarget(10.385, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(85.796), 0.5),
        new TranslationTarget(10.570, 2.963),
        new RotationTarget(Rotation2d.fromDegrees(87.075), 0.5),
        new TranslationTarget(9.995, 3.674),
        new RotationTarget(Rotation2d.fromDegrees(-164.633), 0.5),
        new TranslationTarget(8.855, 2.797),
        new RotationTarget(Rotation2d.fromDegrees(-10.025), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-21.501)));
  }

  static Path mirroredLeftRedMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 0.585), ROT_180),
        new TranslationTarget(8.670, 0.712),
        new RotationTarget(Rotation2d.fromDegrees(95.919), 0.5),
        new TranslationTarget(8.670, 3.606),
        new RotationTarget(Rotation2d.fromDegrees(64.529), 0.5),
        new TranslationTarget(10.404, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(24.079), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)));
  }

  static Path mirroredLeftRedMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)),
        new TranslationTarget(14.623, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(173.062), 0.5),
        new TranslationTarget(13.444, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(172.367), 0.5),
        new TranslationTarget(10.531, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(79.167), 0.5),
        new TranslationTarget(10.726, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(102.987), 0.5),
        new TranslationTarget(8.572, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(179.864), 0.5),
        new TranslationTarget(8.767, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(17.886), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-21.501)));
  }

  static Path mirroredStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(12.096, 0.634), Rotation2d.fromDegrees(-179.384)),
        new TranslationTarget(8.774, 2.602),
        new RotationTarget(ROT_90, 0.5),
        new TranslationTarget(10.301, 2.791),
        new RotationTarget(Rotation2d.fromDegrees(-40.0), 0.5),
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-40.764)));
  }

  static Path mirroredStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-40.764)),
        new TranslationTarget(13.104, 0.697),
        new RotationTarget(ROT_180, 0.5),
        new TranslationTarget(10.774, 0.697),
        new RotationTarget(Rotation2d.fromDegrees(179.148), 0.5),
        new TranslationTarget(10.569, 4.145),
        new RotationTarget(Rotation2d.fromDegrees(69.738), 0.5),
        new TranslationTarget(8.947, 4.413),
        new RotationTarget(Rotation2d.fromDegrees(-145.0), 0.5),
        new TranslationTarget(9.120, 2.697),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-45.0)));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // FLIPPED (Blue Right) paths
  // ─────────────────────────────────────────────────────────────────────────────

  static Path flippedNoStopRightBlueMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 0.585), ROT_0),
        new TranslationTarget(7.451, 0.780),
        new RotationTarget(Rotation2d.fromDegrees(62.727), 0.5),
        new TranslationTarget(7.451, 2.768),
        new RotationTarget(Rotation2d.fromDegrees(101.566), 0.5),
        new TranslationTarget(5.902, 2.534),
        new RotationTarget(Rotation2d.fromDegrees(170.356), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)));
  }

  static Path flippedNoStopRightBlueMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)),
        new TranslationTarget(1.917, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(6.938), 0.5),
        new TranslationTarget(3.096, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(10.764), 0.5),
        new TranslationTarget(6.155, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(94.204), 0.5),
        new TranslationTarget(5.970, 2.963),
        new RotationTarget(Rotation2d.fromDegrees(92.925), 0.5),
        new TranslationTarget(6.545, 3.674),
        new RotationTarget(Rotation2d.fromDegrees(-15.368), 0.5),
        new TranslationTarget(7.685, 2.797),
        new RotationTarget(Rotation2d.fromDegrees(-169.975), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-158.499)));
  }

  static Path flippedRightBlueMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 0.585), ROT_0),
        new TranslationTarget(7.870, 0.712),
        new RotationTarget(Rotation2d.fromDegrees(84.081), 0.5),
        new TranslationTarget(7.870, 3.606),
        new RotationTarget(Rotation2d.fromDegrees(115.471), 0.5),
        new TranslationTarget(6.136, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(155.921), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)));
  }

  static Path flippedRightBlueMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)),
        new TranslationTarget(1.917, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(6.938), 0.5),
        new TranslationTarget(3.096, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(7.633), 0.5),
        new TranslationTarget(6.009, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(100.834), 0.5),
        new TranslationTarget(5.814, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(77.013), 0.5),
        new TranslationTarget(7.968, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(0.136), 0.5),
        new TranslationTarget(7.773, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(162.114), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-158.499)));
  }

  static Path flippedStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(4.444, 0.634), Rotation2d.fromDegrees(-0.616)),
        new TranslationTarget(7.766, 2.602),
        new RotationTarget(ROT_90, 0.5),
        new TranslationTarget(6.239, 2.791),
        new RotationTarget(Rotation2d.fromDegrees(-140.0), 0.5),
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-139.236)));
  }

  static Path flippedStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-139.236)),
        new TranslationTarget(3.436, 0.697),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(5.766, 0.697),
        new RotationTarget(Rotation2d.fromDegrees(0.852), 0.5),
        new TranslationTarget(5.971, 4.145),
        new RotationTarget(Rotation2d.fromDegrees(110.262), 0.5),
        new TranslationTarget(7.593, 4.413),
        new RotationTarget(Rotation2d.fromDegrees(-35.0), 0.5),
        new TranslationTarget(7.420, 2.697),
        new RotationTarget(ROT_180, 0.5),
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-135.0)));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // FLIPPED MIRRORED (Blue Left) paths
  // ─────────────────────────────────────────────────────────────────────────────

  static Path flippedMirroredNoStopLeftBlueMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 7.485), ROT_0),
        new TranslationTarget(7.451, 7.290),
        new RotationTarget(Rotation2d.fromDegrees(-62.727), 0.5),
        new TranslationTarget(7.451, 5.302),
        new RotationTarget(Rotation2d.fromDegrees(-101.566), 0.5),
        new TranslationTarget(5.902, 5.536),
        new RotationTarget(Rotation2d.fromDegrees(-170.356), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)));
  }

  static Path flippedMirroredNoStopLeftBlueMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)),
        new TranslationTarget(1.917, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-6.938), 0.5),
        new TranslationTarget(3.096, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-10.764), 0.5),
        new TranslationTarget(6.155, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-94.204), 0.5),
        new TranslationTarget(5.970, 5.107),
        new RotationTarget(Rotation2d.fromDegrees(-92.925), 0.5),
        new TranslationTarget(6.545, 4.396),
        new RotationTarget(Rotation2d.fromDegrees(15.368), 0.5),
        new TranslationTarget(7.685, 5.273),
        new RotationTarget(Rotation2d.fromDegrees(169.975), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(158.499)));
  }

  static Path flippedMirroredLeftBlueMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 7.485), ROT_0),
        new TranslationTarget(7.870, 7.358),
        new RotationTarget(Rotation2d.fromDegrees(-84.081), 0.5),
        new TranslationTarget(7.870, 4.464),
        new RotationTarget(Rotation2d.fromDegrees(-115.471), 0.5),
        new TranslationTarget(6.136, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-155.921), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)));
  }

  static Path flippedMirroredLeftBlueMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)),
        new TranslationTarget(1.917, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-6.938), 0.5),
        new TranslationTarget(3.096, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-7.633), 0.5),
        new TranslationTarget(6.009, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-100.834), 0.5),
        new TranslationTarget(5.814, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-77.013), 0.5),
        new TranslationTarget(7.968, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-0.136), 0.5),
        new TranslationTarget(7.773, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-162.114), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(158.499)));
  }

  static Path flippedMirroredStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(4.444, 7.436), Rotation2d.fromDegrees(0.616)),
        new TranslationTarget(7.766, 5.468),
        new RotationTarget(ROT_NEG_90, 0.5),
        new TranslationTarget(6.239, 5.279),
        new RotationTarget(Rotation2d.fromDegrees(140.0), 0.5),
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(139.236)));
  }

  static Path flippedMirroredStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(139.236)),
        new TranslationTarget(3.436, 7.373),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(5.766, 7.373),
        new RotationTarget(Rotation2d.fromDegrees(-0.852), 0.5),
        new TranslationTarget(5.971, 3.925),
        new RotationTarget(Rotation2d.fromDegrees(-110.262), 0.5),
        new TranslationTarget(7.593, 3.657),
        new RotationTarget(Rotation2d.fromDegrees(35.0), 0.5),
        new TranslationTarget(7.420, 5.373),
        new RotationTarget(ROT_180, 0.5),
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(135.0)));
  }
}
