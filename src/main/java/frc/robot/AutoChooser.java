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
import frc.robot.autos.NamedAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Alliance-aware autonomous chooser. Combines PathPlanner and BLine autos into a single list and
 * filters the displayed options by the current alliance color.
 */
public class AutoChooser {

  private static final String PATHPLANNER_PREFIX = "[PathPlanner] ";

  private final List<NamedAuto> registeredAutos;
  // Keyed by display name. Populated/refreshed in rebuildChooser() after alliance is known,
  // so PathPlanner's alliance-flip is applied correctly.
  private final Map<String, PathPlannerAuto> pathPlannerAutos = new HashMap<>();
  private final Map<String, Pose2d> autoStartingPoses = new HashMap<>();
  private SendableChooser<NamedAuto> chooser = new SendableChooser<>();
  private Optional<Alliance> previousAlliance = Optional.empty();

  /**
   * @param drivetrain the swerve drivetrain subsystem
   * @param superStructure the superstructure subsystem
   * @param shootAtHubSupplier supplier for the shared "shoot at hub" command
   */
  public AutoChooser(
      CommandSwerveDrivetrain drivetrain,
      SuperStructure superStructure,
      Supplier<Command> shootAtHubSupplier) {
    List<NamedAuto> autos = new ArrayList<>();

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      String displayName = PATHPLANNER_PREFIX + autoName;
      Command autoCommand = AutoBuilder.buildAuto(autoName);
      if (autoCommand instanceof PathPlannerAuto ppAuto) {
        pathPlannerAutos.put(displayName, ppAuto);
      }
      autos.add(new NamedAuto(displayName, autoCommand));
    }

    BLineAutos bLineAutos = new BLineAutos(drivetrain, superStructure, shootAtHubSupplier);
    autos.addAll(bLineAutos.getNamedAutos());

    autos.sort((a, b) -> a.name().compareToIgnoreCase(b.name()));
    registeredAutos = autos;

    rebuildChooser(Optional.empty());
  }

  /** Call periodically while disabled to rebuild the chooser when the alliance changes. */
  public void update() {
    Optional<Alliance> currentAlliance =
        DriverStation.isDSAttached() ? DriverStation.getAlliance() : Optional.empty();
    if (currentAlliance.orElse(null) != previousAlliance.orElse(null)) {
      rebuildChooser(currentAlliance);
    }
    previousAlliance = currentAlliance;
  }

  private static final NamedAuto NONE_AUTO = new NamedAuto("None", Commands.none());

  private void rebuildChooser(Optional<Alliance> alliance) {
    autoStartingPoses.clear();
    pathPlannerAutos.forEach(
        (displayName, ppAuto) -> {
          Pose2d pose = ppAuto.getStartingPose();
          if (pose != null) {
            autoStartingPoses.put(displayName, pose);
          }
        });

    chooser.close();
    chooser = new SendableChooser<>();
    chooser.setDefaultOption(NONE_AUTO.name(), NONE_AUTO);

    for (NamedAuto auto : registeredAutos) {
      if (matchesAlliance(auto.name(), alliance)) {
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

  private boolean matchesAlliance(String name, Optional<Alliance> alliance) {
    if (alliance.isEmpty()) {
      System.out.println("[AutoChooser] No alliance set, allowing: " + name);
      return true;
    }
    boolean mentionsRed = name.contains("Red");
    boolean mentionsBlue = name.contains("Blue");
    if (!mentionsRed && !mentionsBlue) {
      System.out.println("[AutoChooser] Alliance=" + alliance.get() + " | PASS (neutral): " + name);
      return true;
    }
    boolean result;
    if (alliance.get() == Alliance.Red) {
      result = mentionsRed;
    } else {
      result = mentionsBlue;
    }
    System.out.println(
        "[AutoChooser] Alliance="
            + alliance.get()
            + " | mentionsRed="
            + mentionsRed
            + " mentionsBlue="
            + mentionsBlue
            + " | "
            + (result ? "PASS" : "FAIL")
            + ": "
            + name);
    return result;
  }

  /**
   * @return the display name of the currently selected auto, or "None" if nothing is selected
   */
  public String getSelectedName() {
    NamedAuto selected = chooser.getSelected();
    return selected != null ? selected.name() : NONE_AUTO.name();
  }

  /**
   * @return the currently selected autonomous command
   */
  public Command getSelected() {
    NamedAuto selected = chooser.getSelected();
    return selected != null ? selected.auto() : Commands.none();
  }

  /**
   * Returns the starting pose of the currently selected auto, if known. PathPlanner autos provide
   * this via their path data (alliance-flipped). BLine autos and "None" return empty.
   *
   * @return the starting pose, or empty if unavailable
   */
  public Optional<Pose2d> getSelectedStartingPose() {
    NamedAuto selected = chooser.getSelected();
    if (selected == null || selected == NONE_AUTO) {
      return Optional.empty();
    }
    return Optional.ofNullable(autoStartingPoses.get(selected.name()));
  }
}
