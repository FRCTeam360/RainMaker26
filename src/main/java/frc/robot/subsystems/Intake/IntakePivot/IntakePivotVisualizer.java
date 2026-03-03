// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Handles visualization of the intake pivot arm for simulation and debugging. */
public class IntakePivotVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;

  /**
   * Create a new intake pivot visualizer.
   *
   * @param armLength Length of the arm in meters
   */
  public IntakePivotVisualizer(double armLength) {
    // Create mechanism on a 1.0x0.5 canvas with white background
    mechanism = new LoggedMechanism2d(1.0, 0.5, new Color8Bit(Color.kWhite));

    // Create pivot root at center of canvas
    LoggedMechanismRoot2d pivotRoot = mechanism.getRoot("pivot", 0.5, 0.25);

    // Create arm ligament (lime green, 6 pixel thickness)
    // IntakePivot ranges from 0° (stowed, nearly vertical) to 94° (deployed, horizontal)
    // Initial angle is 90° in visualization (pointing up, which represents 0° in mechanism)
    armLigament =
        pivotRoot.append(
            new LoggedMechanismLigament2d(
                "intake arm",
                armLength,
                90.0, // Initial angle (90° visual = 0° mechanism, pointing up)
                6, // Line thickness
                new Color8Bit(Color.kLimeGreen)));
  }

  /**
   * Update the arm visualization with the current angle. Should be called every robot loop with the
   * current arm angle. Only updates when running in simulation for performance safety.
   *
   * @param angleRads Arm angle in radians (0 rad = stowed/vertical, positive = rotating toward
   *     horizontal)
   */
  public void update(double angleRads) {
    // Only update visualization in simulation to ensure zero performance impact on real robot
    if (!RobotBase.isSimulation()) {
      return;
    }

    // Convert radians to degrees and correct the visualization angle
    // Mechanism: 0° = vertical up (stowed), 94° = horizontal right (deployed)
    // Visualization: We want 90° visual when mechanism is at 0°, and 0° visual when at 94°
    // Formula: visual_angle = 90° - mechanism_angle
    double mechanismAngleDegrees = Units.radiansToDegrees(angleRads);
    armLigament.setAngle(90.0 - mechanismAngleDegrees);

    // Log the entire mechanism to AdvantageKit for Advantage Scope visualization
    Logger.recordOutput("IntakePivot/Mechanism2d", mechanism);
  }
}
