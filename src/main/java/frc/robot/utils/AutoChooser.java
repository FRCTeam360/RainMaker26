package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Custom auto chooser that wraps PathPlanner's auto building while tracking the selected auto name.
 * WPILib's {@link SendableChooser} doesn't expose the selected key — only the value. This class
 * maintains a reverse lookup so we always know which auto is selected by name.
 */
public class AutoChooser {
  private final SendableChooser<String> nameChooser = new SendableChooser<>();
  private final Map<String, Command> autoCommands = new LinkedHashMap<>();
  private volatile String selectedName;

  /**
   * Builds the chooser by scanning all PathPlanner auto files and constructing a {@link
   * PathPlannerAuto} for each. Must be called after {@link AutoBuilder#configure}.
   */
  public AutoChooser() {
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    nameChooser.setDefaultOption("None", "None");
    autoCommands.put("None", Commands.none());

    for (String name : autoNames) {
      try {
        autoCommands.put(name, new PathPlannerAuto(name));
        nameChooser.addOption(name, name);
      } catch (Exception e) {
        System.err.println("[AutoChooser] Failed to build auto: " + name);
        e.printStackTrace();
      }
    }

    nameChooser.onChange(name -> selectedName = name);
  }

  /**
   * Returns the underlying {@link SendableChooser} for publishing to SmartDashboard.
   *
   * @return the sendable chooser
   */
  public SendableChooser<String> getSendableChooser() {
    return nameChooser;
  }

  /**
   * Returns the name of the currently selected auto.
   *
   * @return the selected auto name, or "None" if nothing has been selected
   */
  public String getSelectedName() {
    String name = selectedName;
    if (name == null) {
      // No onChange fired yet — read the default
      name = nameChooser.getSelected();
    }
    return name;
  }

  /**
   * Returns the command for the currently selected auto.
   *
   * @return the selected auto command, or {@link Commands#none()} if not found
   */
  public Command getSelected() {
    return autoCommands.getOrDefault(getSelectedName(), Commands.none());
  }
}
