package frc;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.FieldConstants;
import java.util.HashMap;
import org.junit.jupiter.api.Test;

public class TestVision { // Make test cases for isRobotOutOfBounds method
  Vision vision = new Vision(new HashMap<>());

  @Test
  void isRobotOutOfBounds() {
    // X = -1 and Y = 1 should return true
    Pose2d pose = new Pose2d(-1, 1, null);
    boolean isTrue = vision.isPoseOutOfBounds(pose);
    assertTrue(isTrue, "Pose should be out of bounds");
  }

  @Test
  void isRobotOutOfBounds2() {
    // X = 1 and Y = -1 should return true
    Pose2d pose = new Pose2d(1, -1, null);
    boolean isTrue = vision.isPoseOutOfBounds(pose);
    assertTrue(isTrue, "Pose should be out of bounds");
  }

  @Test
  void isRobotOutOfBounds3() {
    Pose2d pose = new Pose2d(FieldConstants.fieldLength + 0.1, 1, null);
    boolean isTrue = vision.isPoseOutOfBounds(pose);
    assertTrue(isTrue, "Pose should be out of bounds");
  }

  @Test
  void isRobotOutOfBounds4() {
    Pose2d pose = new Pose2d(1, FieldConstants.fieldWidth + 0.1, null);
    boolean isTrue = vision.isPoseOutOfBounds(pose);
    assertTrue(isTrue, "Pose should be out of bounds");
  }

  @Test
  void isRobotInBounds() {
    // X = 1 and Y = 1 should return false
    Pose2d pose = new Pose2d(1, 1, null);
    boolean isFalse = vision.isPoseOutOfBounds(pose);
    assertFalse(isFalse, "Pose should be out of bounds");
  }

  @Test
  void isRobotInBounds2() {
    Pose2d pose = new Pose2d(FieldConstants.fieldLength, 1, null);
    boolean isFalse = vision.isPoseOutOfBounds(pose);
    assertFalse(isFalse, "Pose should be in bounds");
  }

  @Test
  void isRobotInBounds3() {
    Pose2d pose = new Pose2d(1, FieldConstants.fieldWidth, null);
    boolean isFalse = vision.isPoseOutOfBounds(pose);
    assertFalse(isFalse, "Pose should be in bounds");
  }
}
