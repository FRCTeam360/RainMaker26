package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mockStatic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

public class TestAllianceFlipUtil {
  @BeforeEach
  void beforeEach() {
    try (MockedStatic<DriverStation> ds = mockStatic(DriverStation.class)) {
      ds.when(DriverStation::getAlliance).thenReturn(Optional.of(DriverStation.Alliance.Red));
      // now test Red behavior
    }
  }

  @Test
  public void blueTestXandY() {
    assertEquals(3.0, AllianceFlipUtil.applyX(3.0), 1e-9);
    assertEquals(2.0, AllianceFlipUtil.applyY(2.0), 1e-9);
  }

  @Test
  public void blueTestTranslation2D() {
    Translation2d t = new Translation2d(3.0, 2.0);
    assertEquals(t, AllianceFlipUtil.apply(t));
  }

  @Test
  public void blueTestRotation2D() {
    Rotation2d r = Rotation2d.fromDegrees(45);
    assertEquals(r, AllianceFlipUtil.apply(r));
  }

  @Test
  public void blueTestPose2D() {
    Pose2d p = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45));
    assertEquals(p, AllianceFlipUtil.apply(p));
  }

  @Test
  public void blueTestTranslation3D() {
    Translation3d t3 = new Translation3d(3.0, 2.0, 1.5);
    assertEquals(t3, AllianceFlipUtil.apply(t3));
  }

  double fieldLength = FieldConstants.fieldLength;
  double fieldWidth = FieldConstants.fieldWidth;

  @Test
  public void redTestXAndY() {
    // not working, somebody else fix please
  }

  @Test
  public void redTestTranslation2D() {
    // Not working, somebody else fix please
  }

  @Test
  public void redTestRotation2D() {
    // Not working, somebody else fix please
  }

  @Test
  public void redTestPose2D() {
    // Wasn't working, somebody else make please
  }

  @Test
  public void redTestTranslation3D() {
    // Not working, somebody else please fix, as I have to work on smtn else
  }
}
