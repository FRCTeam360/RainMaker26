// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Handles visualization of the hood arm for simulation and debugging. */
public class HoodVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d hoodLigament;

  /**
   * Create a new hood visualizer.
   *
   * @param armLength Length of the hood arm in meters
   */
  public HoodVisualizer(double armLength) {
    // Create mechanism on a 1.0x0.5 canvas with white background
    mechanism = new LoggedMechanism2d(1.0, 0.5, new Color8Bit(Color.kWhite));

    // Create pivot root at position that represents the hood pivot point
    // Hood is 12 inches (0.3048m) above the intake pivot
    // Placing it higher on the canvas to visualize the vertical offset
    LoggedMechanismRoot2d pivotRoot = mechanism.getRoot("hood_pivot", 0.5, 0.55);

    // Create hood ligament (orange, 6 pixel thickness)
    // Hood ranges from 0° (horizontal, low angle shot) to 47° (angled up, high arc shot)
    // Initial angle is 0° (pointing to the right, same as intake pivot direction)
    hoodLigament =
        pivotRoot.append(
            new LoggedMechanismLigament2d(
                "hood arm",
                armLength,
                0.0, // Initial angle (0° = horizontal right)
                6, // Line thickness
                new Color8Bit(Color.kOrange)));
  }

  /**
   * Update the hood visualization with the current angle. Should be called every robot loop with
   * the current hood angle. Only updates when running in simulation for performance safety.
   *
   * @param angleRads Hood angle in radians (0 rad = horizontal, positive = angling upward)
   */
  public void update(double angleRads) {
    // Only update visualization in simulation to ensure zero performance impact on real robot
    if (!RobotBase.isSimulation()) {
      return;
    }

    // Convert radians to degrees and set the hood angle
    // Hood: 0° = horizontal right, 47° = angled upward for high arc shots
    hoodLigament.setAngle(Units.radiansToDegrees(angleRads));

    // Log the entire mechanism to AdvantageKit for Advantage Scope visualization
    Logger.recordOutput("Hood/Mechanism2d", mechanism);
  }
}
