package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.BLineAutos;
import frc.robot.autos.NamedAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Alliance-aware autonomous chooser. Combines PathPlanner and BLine autos into a single list and
 * filters the displayed options by the current alliance color.
 */
public class AutoChooser {

  private final List<NamedAuto> registeredAutos;
  private final SendableChooser<Command> chooser = new SendableChooser<>();
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
      autos.add(new NamedAuto("[PathPlanner] " + autoName, AutoBuilder.buildAuto(autoName)));
    }

    BLineAutos bLineAutos = new BLineAutos(drivetrain, superStructure, shootAtHubSupplier);
    autos.addAll(bLineAutos.getNamedAutos());

    autos.sort((a, b) -> a.name().compareToIgnoreCase(b.name()));
    registeredAutos = autos;

    SmartDashboard.putData("Auto Chooser", chooser);
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

  private void rebuildChooser(Optional<Alliance> alliance) {
    chooser.close();

    for (NamedAuto auto : registeredAutos) {
      if (matchesAlliance(auto.name(), alliance)) {
        chooser.addOption(auto.name(), auto.auto());
      }
    }
    SmartDashboard.putData("Auto Chooser", chooser);
  }

  private boolean matchesAlliance(String name, Optional<Alliance> alliance) {
    if (alliance.isEmpty()) return true;
    if (alliance.get() == Alliance.Red) return name.contains("Red");
    if (alliance.get() == Alliance.Blue) return name.contains("Blue");
    return true;
  }

  /**
   * @return the currently selected autonomous command
   */
  public Command getSelected() {
    return chooser.getSelected();
  }
}
