package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Shooter.ShotCalculatorHelpers;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link ShotCalculatorHelpers#shooterFieldVelocity} derived from rigid body kinematics.
 *
 * <p>The velocity of a point P fixed on a rigid body rotating in 2D is:
 *
 * <pre>
 *   v_P = v_O + ω × r_OP
 * </pre>
 *
 * where:
 *
 * <ul>
 *   <li>v_O = velocity of the reference point (robot center) in the field frame
 *   <li>ω = scalar angular velocity (positive = counter-clockwise), treated as ω ẑ
 *   <li>r_OP = position vector from robot center to shooter, rotated into the field frame: r_field
 *       = R(θ) · r_robot
 *   <li>In 2D the cross product reduces to: ω × r = (−ω · r_y, ω · r_x)
 * </ul>
 *
 * <p>Reference: <a
 * href="https://eng.libretexts.org/Bookshelves/Mechanical_Engineering/Introductory_Dynamics:_2D_Kinematics_and_Kinetics_of_Point_Masses_and_Rigid_Bodies_(Steeneken)/03:_Rigid_Body_Dynamics/09:_Kinematics_of_Rigid_Bodies/9.03:_Velocities_in_a_rigid_body">
 * Velocities in a Rigid Body — Engineering LibreTexts</a>
 */
public class TestShotCalculatorHelpers {

  private static final double TOLERANCE = 1e-9;

  /** Creates a robot-to-shooter transform with zero rotation (offset only). */
  private static Transform2d offsetTransform(double xMeters, double yMeters) {
    return new Transform2d(new Translation2d(xMeters, yMeters), Rotation2d.kZero);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Pure translation cases (ω = 0): the ω × r term vanishes entirely,
  // so v_shooter = v_robot regardless of offset or heading.
  // ──────────────────────────────────────────────────────────────────────────

  @Test
  void zeroVelocityZeroRotation_returnsZero() {
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 0.0, 0.0, offsetTransform(0.5, 0.0));

    assertEquals(0.0, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  @Test
  void pureTranslation_matchesRobotVelocity() {
    Translation2d robotVel = new Translation2d(2.0, 1.0);

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            robotVel, 0.0, Math.PI / 4, offsetTransform(0.3, -0.1));

    assertEquals(2.0, result.getX(), TOLERANCE);
    assertEquals(1.0, result.getY(), TOLERANCE);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Pure rotation cases (v_robot = 0): only the ω × r term contributes.
  // ──────────────────────────────────────────────────────────────────────────

  @Test
  void pureRotation_shooterAtCenter_returnsZero() {
    // r = (0,0) → ω × r = (0,0)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 5.0, 0.0, offsetTransform(0.0, 0.0));

    assertEquals(0.0, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  @Test
  void pureRotation_offsetAlongRobotX_headingZero() {
    // θ = 0, r_robot = (0.5, 0) → r_field = (0.5, 0)
    // ω × r = (−2·0, 2·0.5) = (0, 1)
    double offsetXMeters = 0.5;
    double omegaRadPerSec = 2.0;

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), omegaRadPerSec, 0.0, offsetTransform(offsetXMeters, 0.0));

    assertEquals(0.0, result.getX(), TOLERANCE);
    assertEquals(omegaRadPerSec * offsetXMeters, result.getY(), TOLERANCE);
  }

  @Test
  void pureRotation_offsetAlongRobotY_headingZero() {
    // θ = 0, r_robot = (0, 0.4) → r_field = (0, 0.4)
    // ω × r = (−3·0.4, 3·0) = (−1.2, 0)
    double offsetYMeters = 0.4;
    double omegaRadPerSec = 3.0;

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), omegaRadPerSec, 0.0, offsetTransform(0.0, offsetYMeters));

    assertEquals(-omegaRadPerSec * offsetYMeters, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  @Test
  void pureRotation_negativeOmega_reversesDirection() {
    // CW rotation (ω < 0) with shooter in front → rightward (−Y) tangential velocity
    // θ = 0, r_robot = (0.5, 0) → r_field = (0.5, 0)
    // ω × r = (0, −2·0.5) = (0, −1)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), -2.0, 0.0, offsetTransform(0.5, 0.0));

    assertEquals(0.0, result.getX(), TOLERANCE);
    assertEquals(-1.0, result.getY(), TOLERANCE);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Non-zero heading: the robot-frame offset must be rotated into the field
  // frame before applying ω × r.
  //
  //   r_field = R(θ) · r_robot
  //   R(θ) · (x, y) = (x cosθ − y sinθ,  x sinθ + y cosθ)
  // ──────────────────────────────────────────────────────────────────────────

  @Test
  void rotation_headingPiOver2_offsetAlongRobotX() {
    // θ = π/2, r_robot = (0.5, 0)
    // r_field = R(π/2)·(0.5, 0) = (0.5·0 − 0, 0.5·1 + 0) = (0, 0.5)
    // ω × r = (−2·0.5, 2·0) = (−1, 0)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 2.0, Math.PI / 2, offsetTransform(0.5, 0.0));

    assertEquals(-1.0, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  @Test
  void rotation_headingPi_offsetAlongRobotX() {
    // θ = π, r_robot = (0.5, 0)
    // r_field = R(π)·(0.5, 0) = (−0.5, 0)
    // ω × r = (−2·0, 2·(−0.5)) = (0, −1)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 2.0, Math.PI, offsetTransform(0.5, 0.0));

    assertEquals(0.0, result.getX(), TOLERANCE);
    assertEquals(-1.0, result.getY(), TOLERANCE);
  }

  @Test
  void rotation_heading3PiOver2_offsetAlongRobotX() {
    // θ = 3π/2 = −π/2, r_robot = (0.5, 0)
    // r_field = R(3π/2)·(0.5, 0) = (0.5·0 − 0, 0.5·(−1) + 0) = (0, −0.5)
    // ω × r = (−2·(−0.5), 2·0) = (1, 0)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 2.0, 3 * Math.PI / 2, offsetTransform(0.5, 0.0));

    assertEquals(1.0, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  @Test
  void rotation_diagonalOffset_at45DegreeHeading() {
    // θ = π/4, r_robot = (1, 1)
    // r_field = R(π/4)·(1, 1)
    //         = (cos45 − sin45, sin45 + cos45)
    //         = (0, √2)
    // ω × r = (−1·√2, 1·0) = (−√2, 0)
    double sqrt2 = Math.sqrt(2.0);

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), 1.0, Math.PI / 4, offsetTransform(1.0, 1.0));

    assertEquals(-sqrt2, result.getX(), TOLERANCE);
    assertEquals(0.0, result.getY(), TOLERANCE);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Combined translation + rotation
  // v_shooter = v_robot + ω × r_field
  // ──────────────────────────────────────────────────────────────────────────

  @Test
  void combined_translationPlusRotation_headingZero() {
    // v_robot = (1, 0), ω = 2, θ = 0, r_robot = (0.5, 0)
    // r_field = (0.5, 0), ω × r = (0, 1)
    // v_shooter = (1+0, 0+1) = (1, 1)
    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(1.0, 0.0), 2.0, 0.0, offsetTransform(0.5, 0.0));

    assertEquals(1.0, result.getX(), TOLERANCE);
    assertEquals(1.0, result.getY(), TOLERANCE);
  }

  @Test
  void combined_realisticScenario() {
    // Robot driving at (1, 0.5) m/s, ω = 1 rad/s CCW, θ = π/6, shooter at (0.3, 0) in robot frame
    //
    // r_field = R(π/6)·(0.3, 0) = (0.3 cos30°, 0.3 sin30°) = (0.3·√3/2, 0.3·0.5)
    // ω × r_field = (−1·0.15, 1·0.3·√3/2)
    // v_shooter = (1 − 0.15, 0.5 + 0.3·√3/2)
    double theta = Math.PI / 6;
    double offsetX = 0.3;
    double omega = 1.0;

    double rFieldX = offsetX * Math.cos(theta);
    double rFieldY = offsetX * Math.sin(theta);
    double expectedVx = 1.0 + (-omega * rFieldY);
    double expectedVy = 0.5 + (omega * rFieldX);

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(1.0, 0.5), omega, theta, offsetTransform(offsetX, 0.0));

    assertEquals(expectedVx, result.getX(), TOLERANCE);
    assertEquals(expectedVy, result.getY(), TOLERANCE);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Invariant checks: properties that must hold for ANY valid rigid body
  // velocity computation, derived from the cross product definition.
  // ──────────────────────────────────────────────────────────────────────────

  @Test
  void invariant_tangentialSpeedEqualsOmegaTimesRadius() {
    // For pure rotation: |v_P| = |ω| · |r|
    // This follows from |ω × r| = |ω| |r| sin(90°) = |ω| |r|
    double offsetX = 0.3;
    double offsetY = 0.4;
    double radius = Math.hypot(offsetX, offsetY); // 0.5
    double omega = 3.0;
    double theta = 1.2; // arbitrary heading

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), omega, theta, offsetTransform(offsetX, offsetY));

    assertEquals(Math.abs(omega) * radius, result.getNorm(), TOLERANCE);
  }

  @Test
  void invariant_tangentialVelocityPerpendicularToRadius() {
    // For pure rotation: v · r_field = 0  (tangential velocity ⊥ radius)
    // This is a fundamental property of the cross product: (ω × r) · r = 0
    double offsetX = 0.6;
    double offsetY = -0.2;
    double omega = 2.5;
    double theta = 0.8;

    double rFieldX = offsetX * Math.cos(theta) - offsetY * Math.sin(theta);
    double rFieldY = offsetX * Math.sin(theta) + offsetY * Math.cos(theta);

    Translation2d result =
        ShotCalculatorHelpers.shooterFieldVelocity(
            new Translation2d(), omega, theta, offsetTransform(offsetX, offsetY));

    double dotProduct = result.getX() * rFieldX + result.getY() * rFieldY;
    assertEquals(0.0, dotProduct, TOLERANCE);
  }
}
