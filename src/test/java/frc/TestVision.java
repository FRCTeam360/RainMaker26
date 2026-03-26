package frc;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ObjectInputStream.GetField;
import java.util.HashMap;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.FieldConstants;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public class TestVision { //Make test cases for isRobotOutOfBounds method
    Vision vision = new Vision(new HashMap<>());
    
    @Test
    void isRobotOutOfBounds() {
        // X = -1 and Y = 1 should return true
       Pose2d pose = new Pose2d( -1,  1, null);
       boolean isTrue = vision.isPoseOutOfBounds(pose);
       assertTrue(isTrue, "Pose should be out of bounds");
    }

    @Test
    void isRobotOutOfBounds2() {
        // X = 1 and Y = -1 should return true
       Pose2d pose = new Pose2d(1,  -1, null);
       boolean isTrue = vision.isPoseOutOfBounds(pose);
       assertTrue(isTrue, "Pose should be out of bounds");
    }

    @Test
    void isRobotOutOfBounds3() {
       Pose2d pose = new Pose2d(FieldConstants.fieldLength + 0.1,  1, null);
       boolean isTrue = vision.isPoseOutOfBounds(pose);
       assertTrue(isTrue, "Pose should be out of bounds");
    }

    @Test
    void isRobotOutOfBounds4() {
       Pose2d pose = new Pose2d(1,  FieldConstants.fieldWidth + 0.1, null);
       boolean isTrue = vision.isPoseOutOfBounds(pose);
       assertTrue(isTrue, "Pose should be out of bounds");
    }
}
