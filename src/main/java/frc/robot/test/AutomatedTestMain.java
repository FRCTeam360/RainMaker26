// Based on automated test implementation from FRC 2412 (Robototes)
// https://github.com/robototes/REBUILT2026

package frc.robot.test;

import edu.wpi.first.wpilibj.RobotBase;

public final class AutomatedTestMain {
  public static void main(String[] args) {
    RobotBase.startRobot(AutomatedTestRobot::new);
  }
}
