package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.BLineAutos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoChooser {

  private static SendableChooser<Command> ppAutoChooser;
  private static SendableChooser<Command> displayedAutoChooser = new SendableChooser<>();
  private Optional<Alliance> lastAllianceState = Optional.empty();

  public AutoChooser(
      CommandSwerveDrivetrain drivetrain,
      SuperStructure superStructure,
      Supplier<Command> shootAtHubSupplier) {
    ppAutoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", displayedAutoChooser);
    BLineAutos bLineAutos = new BLineAutos(drivetrain, superStructure, shootAtHubSupplier);
    bLineAutos.registerAutos(ppAutoChooser);
  }

  public void update() {
    if (DriverStation.isDSAttached()) {
      if (!DriverStation.getAlliance().equals(lastAllianceState)) {

        lastAllianceState = DriverStation.getAlliance();
        displayedAutoChooser.close();
        List<String> autoNames = AutoBuilder.getAllAutoNames();

        for (String autoName : autoNames) {
          if (lastAllianceState.equals(Optional.of(Alliance.Red))) {
            if (autoName.contains("Red")) {
              PathPlannerAuto auto = new PathPlannerAuto(autoName);
              displayedAutoChooser.addOption(auto.getName(), auto);
            }
          } else if (lastAllianceState.equals(Optional.of(Alliance.Blue))) {
            if (autoName.contains("Blue")) {
              PathPlannerAuto auto = new PathPlannerAuto(autoName);
              displayedAutoChooser.addOption(auto.getName(), auto);
            }
          } else {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            displayedAutoChooser.addOption(auto.getName(), auto);
          }
        }
        SmartDashboard.putData("Auto Chooser", displayedAutoChooser);
      }
    }
  }

  public Command getSelected() {
    return displayedAutoChooser.getSelected();
  }
}
