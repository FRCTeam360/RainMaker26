package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BLineAutos;
import frc.robot.autos.NamedAutoWithPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Alliance-aware autonomous chooser. Combines PathPlanner and BLine autos into a single list and
 * filters the displayed options by the current alliance color and the selected field zone (based
 * on each auto's starting Y position).
 */
public class AutoChooser {

  private static final String PATH_PLANNER_PREFIX = "[PathPlanner] ";

  /** Field zones used to group autos by their starting Y position. */
  public enum AutoZone {
    ALL("All"),
    LEFT_TRENCH("Left Trench"),
    LEFT_BUMP("Left Bump"),
    HUB("Hub"),
    RIGHT_BUMP("Right Bump"),
    RIGHT_TRENCH("Right Trench");

    private final String displayName;

    AutoZone(String displayName) {
      this.displayName = displayName;
    }

    public String getDisplayName() {
      return displayName;
    }
  }

  private final List<NamedAutoWithPose> registeredAutos;
  private final Map<String, Pose2d> autoStartingPoses = new HashMap<>();
  private SendableChooser<NamedAutoWithPose> chooser = new SendableChooser<>();
  private final SendableChooser<AutoZone> zoneChooser = new SendableChooser<>();
  private Optional<Alliance> previousAlliance = Optional.empty();
  private AutoZone previousZone = AutoZone.ALL;

  /**
   * @param drivetrain the swerve drivetrain subsystem
   * @param superStructure the superstructure subsystem
   * @param shootAtHubSupplier supplier for the shared "shoot at hub" command
   */
  public AutoChooser(
      CommandSwerveDrivetrain drivetrain,
      SuperStructure superStructure,
      Supplier<Command> shootAtHubSupplier) {
    List<NamedAutoWithPose> autos = new ArrayList<>();

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      String displayName = PATH_PLANNER_PREFIX + autoName;
      Command autoCommand = AutoBuilder.buildAuto(autoName);
      if (autoCommand instanceof PathPlannerAuto auto) {
        Pose2d startingPose = auto.getStartingPose();
        if (startingPose != null) {
          autoStartingPoses.put(displayName, startingPose);
          autos.add(new NamedAutoWithPose(displayName, autoCommand, startingPose));
        } else {
          Logger.recordOutput("PathPlannerAutos/MissingStartPose", autoName);
        }
      } else {
        Logger.recordOutput("PathPlannerAutos/FailedCast", autoName);
      }
    }

    BLineAutos bLineAutos = new BLineAutos(drivetrain, superStructure, shootAtHubSupplier);
    for (NamedAutoWithPose bLineAuto : bLineAutos.getNamedAutos()) {
      autoStartingPoses.put(bLineAuto.name(), bLineAuto.startingPose());
      autos.add(bLineAuto);
    }

    autos.sort((a, b) -> a.name().compareToIgnoreCase(b.name()));
    registeredAutos = autos;

    for (AutoZone zone : AutoZone.values()) {
      if (zone == AutoZone.ALL) {
        zoneChooser.setDefaultOption(zone.getDisplayName(), zone);
      } else {
        zoneChooser.addOption(zone.getDisplayName(), zone);
      }
    }
    SmartDashboard.putData("Auto Zone Filter", zoneChooser);

    rebuildChooser(Optional.empty(), AutoZone.ALL);
  }

  /**
   * Call periodically while disabled to rebuild the chooser when the alliance or selected zone
   * changes.
   */
  public void update() {
    Optional<Alliance> currentAlliance =
        DriverStation.isDSAttached() ? DriverStation.getAlliance() : Optional.empty();
    AutoZone currentZone = zoneChooser.getSelected();
    if (currentZone == null) {
      currentZone = AutoZone.ALL;
    }

    boolean allianceChanged = currentAlliance.orElse(null) != previousAlliance.orElse(null);
    boolean zoneChanged = currentZone != previousZone;
    if (allianceChanged || zoneChanged) {
      rebuildChooser(currentAlliance, currentZone);
    }
    previousAlliance = currentAlliance;
    previousZone = currentZone;
  }

  private static final NamedAutoWithPose NONE_AUTO =
      new NamedAutoWithPose("None", Commands.none(), new Pose2d());

  private void rebuildChooser(Optional<Alliance> alliance, AutoZone zone) {
    chooser.close();
    chooser = new SendableChooser<>();
    chooser.setDefaultOption(NONE_AUTO.name(), NONE_AUTO);

    for (NamedAutoWithPose auto : registeredAutos) {
      if (matchesAlliance(auto, alliance) && matchesZone(auto, zone)) {
        chooser.addOption(auto.name(), auto);
      }
    }
    SmartDashboard.putData("Auto Chooser", chooser);
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable("Auto Chooser")
        .getEntry("selected")
        .setString("None");
  }

  private boolean matchesAlliance(NamedAutoWithPose auto, Optional<Alliance> alliance) {
    String name = auto.name();
    if (alliance.isEmpty()) {
      System.out.println("[AutoChooser] No alliance set, adding: " + name);
      return true;
    }
    Pose2d startingPose = auto.startingPose();
    double startX = startingPose.getX();
    double startY = startingPose.getY();
    if (startX == 0.0 && startY == 0.0) {
      System.out.println("[AutoChooser] Fallback start pose, adding for both alliances: " + name);
      return true;
    }
    double midfield = FieldConstants.fieldLength / 2.0;
    boolean onBlueSide = startX < midfield;
    boolean result = (alliance.get() == Alliance.Blue) ? onBlueSide : !onBlueSide;
    System.out.println(
        "[AutoChooser] Alliance="
            + alliance.get()
            + " | startX="
            + startX
            + " midfield="
            + midfield
            + " onBlueSide="
            + onBlueSide
            + " | "
            + (result ? "ADDED" : "EXCLUDED")
            + ": "
            + name);
    return result;
  }

  private boolean matchesZone(NamedAutoWithPose auto, AutoZone zone) {
    if (zone == AutoZone.ALL) {
      return true;
    }
    Pose2d startingPose = auto.startingPose();
    double startX = startingPose.getX();
    double startY = startingPose.getY();
    if (startX == 0.0 && startY == 0.0) {
      return true;
    }
    return determineZone(startY) == zone;
  }

  /**
   * Maps a starting Y coordinate to its corresponding {@link AutoZone}. Boundaries are taken from
   * {@link FieldConstants.LinesHorizontal}; the 12-inch gap between each bump and trench is folded
   * into the trench zone.
   */
  private static AutoZone determineZone(double startY) {
    if (startY < FieldConstants.LinesHorizontal.rightBumpRailSide) {
      return AutoZone.RIGHT_TRENCH;
    } else if (startY < FieldConstants.LinesHorizontal.rightBumpHubSide) {
      return AutoZone.RIGHT_BUMP;
    } else if (startY <= FieldConstants.LinesHorizontal.leftBumpHubSide) {
      return AutoZone.HUB;
    } else if (startY <= FieldConstants.LinesHorizontal.leftBumpRailSide) {
      return AutoZone.LEFT_BUMP;
    } else {
      return AutoZone.LEFT_TRENCH;
    }
  }

  /**
   * @return the currently selected autonomous command
   */
  public Command getSelected() {
    NamedAutoWithPose selected = chooser.getSelected();
    return selected != null ? selected.auto() : NONE_AUTO.auto();
  }

  public String getSelectedName() {
    NamedAutoWithPose selected = chooser.getSelected();
    return selected != null ? selected.name() : NONE_AUTO.name();
  }

  public Optional<Pose2d> getSelectedStartingPose() {
    NamedAutoWithPose selected = chooser.getSelected();
    if (selected == null) {
      return Optional.empty();
    }
    return Optional.ofNullable(autoStartingPoses.get(selected.name()));
  }
}
