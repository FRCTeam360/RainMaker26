package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Pure-math helpers for shot calculation, extracted for testability. */
public class ShotCalculatorHelpers {

  /**
   * Computes the field-relative velocity of the shooter mechanism. Combines the robot's
   * translational velocity (already in the field frame) with the rotational contribution of the
   * shooter's offset from the robot center.
   *
   * @param robotFieldVelocity the robot's translational velocity rotated into the field frame
   * @param omegaRadPerSec the robot's angular velocity in radians per second
   * @param robotAngleRad the robot's heading in radians
   * @param robotToShooter the transform from the robot center to the shooter
   * @return a Translation2d representing the shooter's field-relative velocity (x, y) in m/s
   */
  public static Translation2d shooterFieldVelocity(
      Translation2d robotFieldVelocity,
      double omegaRadPerSec,
      double robotAngleRad,
      Transform2d robotToShooter) {
    double shooterOffsetX = robotToShooter.getX();
    double shooterOffsetY = robotToShooter.getY();

    double shooterVelXMps =
        robotFieldVelocity.getX()
            + omegaRadPerSec
                * (shooterOffsetY * (-1.0) * Math.cos(robotAngleRad)
                    - shooterOffsetX * Math.sin(robotAngleRad));
    double shooterVelYMps =
        robotFieldVelocity.getY()
            + omegaRadPerSec
                * (shooterOffsetX * Math.cos(robotAngleRad)
                    - shooterOffsetY * Math.sin(robotAngleRad));

    return new Translation2d(shooterVelXMps, shooterVelYMps);
  }
}
