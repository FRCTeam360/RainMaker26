// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Handles visualization of the intake pivot arm for simulation and debugging. */
public class TurretVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;

  /**
   * Create a new intake pivot visualizer.
   *
   * @param armLength Length of the arm in meters
   */
  public TurretVisualizer(double armLength) {
    // Create mechanism on a 3x3 canvas with white background
    mechanism = new LoggedMechanism2d(1.0, 0.5, new Color8Bit(Color.kWhite));

    // Create pivot root at center of canvas
    LoggedMechanismRoot2d pivotRoot = mechanism.getRoot("pivot", 0.0, 0.25);

    // Create arm ligament (lime green, 6 pixel thickness)
    armLigament =
        pivotRoot.append(
            new LoggedMechanismLigament2d(
                "intake arm",
                armLength,
                180.0, // Initial angle (pointing up)
                6, // Line thickness
                new Color8Bit(Color.kLimeGreen)));
  }

  /**
   * Update the arm visualization with the current angle. Should be called every robot loop with the
   * current arm angle.
   *
   * @param angleRads Arm angle in radians
   */
  public void update(double angleRads) {
    // Convert radians to degrees and set the arm angle
    armLigament.setAngle(Units.radiansToDegrees(angleRads));

    // Log the entire mechanism to AdvantageKit for Advantage Scope visualization
    Logger.recordOutput("Turret/Mechanism2d", mechanism);
  }
}
