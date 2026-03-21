package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A SendableChooser wrapper that filters PathPlanner autos by alliance color and displays the
 * selected auto's paths on a Field2d widget. Autos are expected to be prefixed with "Red" or
 * "Blue". Call {@link #update()} periodically while disabled so the chooser reflects the correct
 * alliance when FMS connects.
 */
public class AllianceAutoChooser {
  private static final String SMARTDASHBOARD_KEY = "Auto Chooser";
  private static final String FIELD_KEY = "Auto Preview";

  private SendableChooser<String> chooser;
  private Alliance lastAlliance;
  private String lastSelectedAuto;
  private final Field2d field;
  private StringSubscriber selectedSubscriber;

  /** Creates a new AllianceAutoChooser and publishes it to SmartDashboard. */
  public AllianceAutoChooser() {
    chooser = new SendableChooser<>();
    field = new Field2d();
    SmartDashboard.putData(SMARTDASHBOARD_KEY, chooser);
    SmartDashboard.putData(FIELD_KEY, field);
    subscribeToSelection();
    update();
  }

  /** Subscribes to the NetworkTables selected value for the chooser. */
  private void subscribeToSelection() {
    selectedSubscriber =
        NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/" + SMARTDASHBOARD_KEY + "/selected")
            .subscribe("");
  }

  /**
   * Checks the current alliance and rebuilds the chooser if the alliance has changed. Safe to call
   * every loop cycle — it short-circuits when the alliance hasn't changed.
   */
  public void update() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance currentAlliance = allianceOpt.orElse(Alliance.Blue);

    if (currentAlliance != lastAlliance) {
      lastAlliance = currentAlliance;
      lastSelectedAuto = null;

      chooser.close();
      chooser = new SendableChooser<>();

      String prefix = currentAlliance == Alliance.Red ? "Red" : "Blue";
      List<String> allAutoNames = AutoBuilder.getAllAutoNames();

      boolean first = true;
      for (String autoName : allAutoNames) {
        if (autoName.startsWith(prefix)) {
          if (first) {
            chooser.setDefaultOption(autoName, autoName);
            first = false;
          } else {
            chooser.addOption(autoName, autoName);
          }
        }
      }

      SmartDashboard.putData(SMARTDASHBOARD_KEY, chooser);

      // Resubscribe since the chooser was recreated
      selectedSubscriber.close();
      subscribeToSelection();
    }
  }

  /**
   * Checks if the selected auto has changed via NetworkTables and updates the field preview. Call
   * this periodically (e.g., in {@code disabledPeriodic()}).
   */
  public void checkForAutoChange() {
    String selectedAuto = selectedSubscriber.get();
    if (selectedAuto.isEmpty()) {
      selectedAuto = chooser.getSelected();
    }
    if (selectedAuto != null && !selectedAuto.equals(lastSelectedAuto)) {
      lastSelectedAuto = selectedAuto;
      System.out.println("[AllianceAutoChooser] Auto changed to: " + selectedAuto);
      updateFieldPreview(selectedAuto);
    }
  }

  /**
   * Loads the paths from the selected auto and displays them on the field widget.
   *
   * @param autoName the name of the auto to preview
   */
  private void updateFieldPreview(String autoName) {
    // Clear previous path objects
    field.getObject("auto-preview").setPoses();

    try {
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      List<Pose2d> allPoses = new ArrayList<>();

      for (PathPlannerPath path : paths) {
        allPoses.addAll(path.getPathPoses());
      }

      FieldObject2d previewObject = field.getObject("auto-preview");
      previewObject.setPoses(allPoses);
    } catch (Exception e) {
      System.err.println("[AllianceAutoChooser] Failed to load paths for auto: " + autoName);
      e.printStackTrace();
    }
  }

  /**
   * Builds and returns the currently selected autonomous command.
   *
   * @return the selected auto command, or {@link Commands#none()} if nothing is selected
   */
  public Command getSelected() {
    String selectedAutoName = chooser.getSelected();
    if (selectedAutoName == null) {
      return Commands.none();
    }
    return AutoBuilder.buildAuto(selectedAutoName);
  }
}
