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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Alliance-aware autonomous chooser. Combines PathPlanner and BLine autos into a single list and
 * filters the displayed options by the current alliance color.
 */
public class AutoChooser {

  private static final String PATH_PLANNER_PREFIX = "[PathPlanner] ";

  private final List<NamedAutoWithPose> registeredAutos;
  private final Map<String, Pose2d> autoStartingPoses = new HashMap<>();
  private SendableChooser<NamedAutoWithPose> chooser = new SendableChooser<>();
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

  private static final NamedAutoWithPose NONE_AUTO =
      new NamedAutoWithPose("None", Commands.none(), new Pose2d());

  private void rebuildChooser(Optional<Alliance> alliance) {
    chooser.close();
    chooser = new SendableChooser<>();
    chooser.setDefaultOption(NONE_AUTO.name(), NONE_AUTO);

    for (NamedAutoWithPose auto : registeredAutos) {
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
