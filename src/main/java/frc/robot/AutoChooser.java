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
    if (DriverStation.isDSAttached()) {
      if (!DriverStation.getAlliance().equals(lastAllianceState)) {

        displayedAutoChooser.close();

        for (NamedAuto auto : allAutos) {
          if (lastAllianceState.equals(Optional.of(Alliance.Red))) {
            if (auto.name().contains("Red")) {
              displayedAutoChooser.addOption(auto.name(), auto.auto());
            }
          } else if (lastAllianceState.equals(Optional.of(Alliance.Blue))) {
            if (auto.name().contains("Blue")) {
              displayedAutoChooser.addOption(auto.name(), auto.auto());
            }
          } else {
            displayedAutoChooser.addOption(auto.name(), auto.auto());
          }
        }
        SmartDashboard.putData("Auto Chooser", displayedAutoChooser);
      }
      lastAllianceState = DriverStation.getAlliance();
    }
  }

  public Command getSelected() {
    return displayedAutoChooser.getSelected();
  }
}
