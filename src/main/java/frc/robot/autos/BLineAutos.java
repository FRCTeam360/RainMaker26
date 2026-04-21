package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeStateMachine.IntakeWantedStates;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperWantedStates;
import frc.robot.utils.CommandLogger;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * BLine-based autonomous routines. Composes paths from {@link BLinePaths} into full auto commands
 * and registers them in the auto chooser.
 */
public class BLineAutos {

  private final FollowPath.Builder pathBuilder;
  private final SuperStructure superStructure;
  private final Supplier<Command> shootAtHubSupplier;

  /**
   * @param drivetrain the swerve drivetrain subsystem
   * @param superStructure the superstructure subsystem
   * @param shootAtHubSupplier supplier for the shared "shoot at hub" command
   */
  public BLineAutos(
      CommandSwerveDrivetrain drivetrain,
      SuperStructure superStructure,
      Supplier<Command> shootAtHubSupplier) {
    this.superStructure = superStructure;
    this.shootAtHubSupplier = shootAtHubSupplier;
    this.pathBuilder = drivetrain.createBLinePathBuilder();
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Coordinate transformations
  // ─────────────────────────────────────────────────────────────────────────────

  // ─────────────────────────────────────────────────────────────────────────────
  // Shared command factories
  // ─────────────────────────────────────────────────────────────────────────────

  private Command loggedCommand(Command command, String name) {
    return CommandLogger.logCommand(command, name);
  }

  private Command defaultCmd() {
    return loggedCommand(superStructure.setStateCommand(SuperWantedStates.DEFAULT), "default");
  }

  private Command basicIntake() {
    return loggedCommand(
        superStructure.setIntakeStateCommand(IntakeWantedStates.INTAKING), "basic intake");
  }

  private Command agitateIntake() {
    return loggedCommand(
        superStructure.setIntakeStateCommand(IntakeWantedStates.AGITATING), "agitate intake");
  }

  /** The full "shoot at hub" command, shared with PathPlanner autos. */
  private Command shootAtHub() {
    return shootAtHubSupplier.get();
  }

  /** Shortcut: follow path with intake deployed after a short delay. */
  private Command pathWithIntake(Path path) {
    return followPath(path).deadlineFor(Commands.waitSeconds(0.05).andThen(basicIntake()));
  }

  /** Shortcut: follow path with immediate intake. */
  private Command pathWithImmediateIntake(Path path) {
    return followPath(path).deadlineFor(basicIntake());
  }

  private Command followPath(Path path) {
    return pathBuilder.build(path);
  }

  private static Path flippedPath(String pathName) {
    Path path = new Path(pathName);
    path.flip();
    return path;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Auto routine compositions
  // ─────────────────────────────────────────────────────────────────────────────

  private NamedAutoWithPose buildTwoSwipeAuto(String autoName) {
    Path firstPath = new Path(autoName + " First Swipe");
    PathElement element = firstPath.getElement(0);
    Pose2d pose = null;
    if (element instanceof Waypoint waypoint) {
      pose =
          new Pose2d(
              waypoint.translationTarget().translation(), waypoint.rotationTarget().rotation());
    } else {
      throw new IllegalStateException(
          "bLine auto "
              + autoName
              + " first path element is not of type waypoint. Cannot determine starting pose. Check bLine JSON file.");
    }
    // TODO URGENT verify if we want the delay in deploying the intake
    Command command =
        pathWithIntake(firstPath)
            .andThen(shootAtHub())
            .andThen(pathWithImmediateIntake(new Path(autoName + " Second Swipe")))
            .andThen(shootAtHub());

    return new NamedAutoWithPose(autoName, command, pose);
  }

  private NamedAutoWithPose buildAutoOrNone(
      String autoName, Function<String, NamedAutoWithPose> autoFactory) {
    try {
      return autoFactory.apply(autoName);
    } catch (Exception e) {
      Logger.recordOutput("BLineAutos/MissingPaths/" + autoName, e.getMessage());
      return new NamedAutoWithPose("!!!NO PATH!!! " + autoName, Commands.none(), new Pose2d());
    }
  }

  /** Builds all BLine auto routines using generated path variants. */
  private List<NamedAutoWithPose> buildAutos() {
    return List.of(
        buildAutoOrNone("FLIPPED Blue Right Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("FLIPPED MIRRORED Blue Left Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("MASTER Red Right Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("MIRRORED Red Left Aggressive", this::buildTwoSwipeAuto));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Auto chooser registration
  // ─────────────────────────────────────────────────────────────────────────────

  /**
   * Returns all BLine auto routines as named autos with a [BLine] prefix so they appear alongside
   * PathPlanner autos for A/B testing.
   */
  public List<NamedAutoWithPose> getNamedAutos() {
    return buildAutos().stream()
        .map(
            auto ->
                new NamedAutoWithPose("[BLine] " + auto.name(), auto.auto(), auto.startingPose()))
        .toList();
  }
}
