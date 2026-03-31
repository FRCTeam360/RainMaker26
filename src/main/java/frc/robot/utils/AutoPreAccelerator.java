package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Map;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/**
 * Fires a single swerve setControl() frame directly in autonomousInit(), bypassing the command
 * scheduler pipeline to eliminate the 1-2 cycle (20-40ms) delay before the first drive command.
 *
 * <p>For immediate-start autos (no initial WaitCommand), this sends an ApplyRobotSpeeds request
 * with the first path's initial velocity — giving the motors a head start on acceleration.
 *
 * <p>For delayed-start autos (with an initial WaitCommand), this sends a PointWheelsAt request to
 * pre-position the steer modules at the first path's heading during the wait.
 *
 * <p>The command scheduler takes ownership on the very next robotPeriodic() cycle, seamlessly
 * overwriting this single-frame command.
 */
public class AutoPreAccelerator {

  /** Parsed info about an auto's first path and whether it starts with a wait. */
  private record AutoStartInfo(String firstPathName, boolean startsWithWait) {}

  private static final Map<String, AutoStartInfo> autoStartInfoMap = new HashMap<>();

  private static final SwerveRequest.ApplyRobotSpeeds applySpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private static final SwerveRequest.PointWheelsAt pointWheelsRequest =
      new SwerveRequest.PointWheelsAt();

  private static final double CHASSIS_SPEEDS_DISCRETIZE_PERIOD_SECONDS = 0.020;
  private static final double MIN_VELOCITY_THRESHOLD_MPS = 0.01;

  /**
   * Scans all .auto files in the deploy directory and builds a map of auto name to first-path info.
   * Should be called once during robot initialization, after PathProvider.initialize().
   */
  public static void initialize() {
    File autoDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    File[] autoFiles = autoDir.listFiles((dir, name) -> name.endsWith(".auto"));

    if (autoFiles == null) {
      System.err.println("[AutoPreAccelerator] No auto files found in deploy directory");
      return;
    }

    JSONParser parser = new JSONParser();

    for (File file : autoFiles) {
      String autoName = file.getName().replaceFirst("\\.auto$", "");
      try {
        String content = new String(Files.readAllBytes(file.toPath()));
        JSONObject root = (JSONObject) parser.parse(content);
        AutoStartInfo info = parseAutoStartInfo(root);
        if (info != null) {
          autoStartInfoMap.put(autoName, info);
        }
      } catch (IOException | ParseException e) {
        System.err.println("[AutoPreAccelerator] Failed to parse auto: " + autoName);
        e.printStackTrace();
      }
    }

    System.out.println(
        "[AutoPreAccelerator] Parsed " + autoStartInfoMap.size() + " auto start configs");
  }

  /**
   * Parses an .auto JSON to find the first path name and whether the auto starts with a wait.
   *
   * @param root the parsed .auto JSON root object
   * @return the start info, or null if no path was found
   */
  private static AutoStartInfo parseAutoStartInfo(JSONObject root) {
    JSONObject command = (JSONObject) root.get("command");
    if (command == null) return null;

    JSONObject data = (JSONObject) command.get("data");
    if (data == null) return null;

    JSONArray commands = (JSONArray) data.get("commands");
    if (commands == null || commands.isEmpty()) return null;

    boolean startsWithWait = false;
    String firstPathName = null;

    for (Object cmdObj : commands) {
      JSONObject cmd = (JSONObject) cmdObj;
      String type = (String) cmd.get("type");

      if ("wait".equals(type) && firstPathName == null) {
        startsWithWait = true;
        continue;
      }

      if ("named".equals(type) && firstPathName == null) {
        // Named commands (like "default") are instant — skip past them
        continue;
      }

      // Look for a path in this command or its children
      firstPathName = findFirstPathName(cmd);
      if (firstPathName != null) break;
    }

    if (firstPathName == null) return null;
    return new AutoStartInfo(firstPathName, startsWithWait);
  }

  /**
   * Recursively searches a command JSON node for the first path reference.
   *
   * @param cmd the command JSON object
   * @return the path name, or null if none found
   */
  private static String findFirstPathName(JSONObject cmd) {
    String type = (String) cmd.get("type");
    JSONObject data = (JSONObject) cmd.get("data");
    if (data == null) return null;

    if ("path".equals(type)) {
      return (String) data.get("pathName");
    }

    // Search children (deadline, sequential, parallel groups)
    JSONArray children = (JSONArray) data.get("commands");
    if (children != null) {
      for (Object childObj : children) {
        String found = findFirstPathName((JSONObject) childObj);
        if (found != null) return found;
      }
    }

    return null;
  }

  /**
   * Fires a single pre-acceleration frame to the drivetrain based on the selected auto. Call this
   * from autonomousInit() after scheduling the auto command.
   *
   * <p>For immediate-start autos: sends initial velocity to begin motor acceleration. For
   * delayed-start autos: pre-positions steer modules to the first path heading.
   *
   * @param selectedAutoName the name of the selected auto (from the chooser)
   * @param drivetrain the swerve drivetrain to command
   */
  public static void preAccelerate(String selectedAutoName, CommandSwerveDrivetrain drivetrain) {
    double t0 = Logger.getTimestamp() / 1.0e6;

    AutoStartInfo info = autoStartInfoMap.get(selectedAutoName);
    if (info == null) {
      Logger.recordOutput("AutoPreAccel/Status", "no_info_for_auto");
      return;
    }

    PathPlannerPath firstPath = PathProvider.fromPathFile(info.firstPathName());
    if (firstPath == null) {
      Logger.recordOutput("AutoPreAccel/Status", "path_not_found");
      return;
    }

    Rotation2d initialHeading = firstPath.getInitialHeading();
    Logger.recordOutput("AutoPreAccel/InitialHeadingDeg", initialHeading.getDegrees());
    Logger.recordOutput("AutoPreAccel/StartsWithWait", info.startsWithWait());

    if (!info.startsWithWait()) {
      // Immediate-start auto: send initial velocity if the path starts with nonzero speed
      IdealStartingState startState = firstPath.getIdealStartingState();
      double startVelocityMps = (startState != null) ? startState.velocityMPS() : 0.0;

      if (startVelocityMps > MIN_VELOCITY_THRESHOLD_MPS) {
        // Path starts with velocity — pre-accelerate in the path's initial direction
        Translation2d fieldVelocity = new Translation2d(startVelocityMps, initialHeading);
        Rotation2d robotHeading =
            (startState != null) ? startState.rotation() : drivetrain.getPosition().getRotation();
        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldVelocity.getX(), fieldVelocity.getY(), 0.0, robotHeading);

        drivetrain.setControl(
            applySpeedsRequest
                .withSpeeds(
                    ChassisSpeeds.discretize(speeds, CHASSIS_SPEEDS_DISCRETIZE_PERIOD_SECONDS))
                .withDriveRequestType(DriveRequestType.Velocity));

        Logger.recordOutput("AutoPreAccel/Status", "velocity_sent");
        Logger.recordOutput("AutoPreAccel/VelocityMps", startVelocityMps);
      } else {
        // Path starts from rest — pre-point wheels
        drivetrain.setControl(pointWheelsRequest.withModuleDirection(initialHeading));
        Logger.recordOutput("AutoPreAccel/Status", "point_wheels_rest_start");
      }
    } else {
      // Delayed-start auto: pre-position steer modules during the wait
      drivetrain.setControl(pointWheelsRequest.withModuleDirection(initialHeading));
      Logger.recordOutput("AutoPreAccel/Status", "point_wheels_delayed_start");
    }

    double t1 = Logger.getTimestamp() / 1.0e6;
    Logger.recordOutput("AutoPreAccel/ExecutionTimeMs", (t1 - t0) * 1000.0);
  }
}
