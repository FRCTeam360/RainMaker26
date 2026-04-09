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

public class AutoChooser {

  private final List<NamedAuto> allAutos;
  private final SendableChooser<Command> displayedAutoChooser = new SendableChooser<>();
  private Optional<Alliance> lastAllianceState = Optional.empty();

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
    allAutos = autos;

    SmartDashboard.putData("Auto Chooser", displayedAutoChooser);
  }

  public void update() {
    Optional<Alliance> currentAlliance =
        DriverStation.isDSAttached() ? DriverStation.getAlliance() : Optional.empty();
    if (currentAlliance.orElse(null) != lastAllianceState.orElse(null)) {
      updateAutoChooser(currentAlliance);
    }
    lastAllianceState = currentAlliance;
  }

  private void updateAutoChooser(Optional<Alliance> currentAlliance) {
    displayedAutoChooser.close();

    for (NamedAuto auto : allAutos) {
      if (matchesAlliance(auto.name(), currentAlliance)) {
        displayedAutoChooser.addOption(auto.name(), auto.auto());
      }
    }
    SmartDashboard.putData("Auto Chooser", displayedAutoChooser);
  }

  private boolean matchesAlliance(String name, Optional<Alliance> alliance) {
    if (alliance.isEmpty()) return true;
    if (alliance.get() == Alliance.Red) return name.contains("Red");
    if (alliance.get() == Alliance.Blue) return name.contains("Blue");
    return true;
  }

  public Command getSelected() {
    return displayedAutoChooser.getSelected();
  }
}
