package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

/**
 * Pre-loads all PathPlanner path files into memory at startup to avoid disk I/O and JIT compilation
 * delays during autonomous.
 *
 * <p>Inspired by FRC Team 5468's PathProvider implementation.
 *
 * @see <a href="https://github.com/FRC5468/FRC2025">FRC 5468 - FRC2025</a>
 */
public class PathProvider {
  private static final Map<String, PathPlannerPath> preloadedPaths = new HashMap<>();

  /**
   * Scans the deploy directory for all .path files and pre-loads them into memory. Should be called
   * once during robot initialization.
   */
  public static void initialize() {
    File pathDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
    File[] pathFiles = pathDir.listFiles((dir, name) -> name.endsWith(".path"));

    if (pathFiles == null) {
      System.err.println("[PathProvider] No path files found in deploy directory");
      return;
    }

    for (File file : pathFiles) {
      String pathName = file.getName().replaceFirst("\\.path$", "");
      try {
        preloadedPaths.put(pathName, PathPlannerPath.fromPathFile(pathName));
      } catch (Exception e) {
        System.err.println("[PathProvider] Failed to load path: " + pathName);
        e.printStackTrace();
      }
    }

    System.out.println("[PathProvider] Pre-loaded " + preloadedPaths.size() + " paths");
  }

  /**
   * Gets a pre-loaded path by name. Falls back to loading from disk if not cached.
   *
   * @param pathName the name of the path (without .path extension)
   * @return the loaded PathPlannerPath, or null if loading fails
   */
  public static PathPlannerPath fromPathFile(String pathName) {
    PathPlannerPath path = preloadedPaths.get(pathName);

    if (path == null) {
      System.err.println(
          "[PathProvider] Path not preloaded: " + pathName + ". Loading from disk...");
      try {
        path = PathPlannerPath.fromPathFile(pathName);
        preloadedPaths.put(pathName, path);
      } catch (Exception e) {
        System.err.println("[PathProvider] Failed to load path from disk: " + pathName);
        e.printStackTrace();
      }
    }

    return path;
  }
}
