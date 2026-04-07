package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeStateMachine.IntakeWantedStates;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperInternalStates;
import frc.robot.subsystems.SuperStructure.SuperWantedStates;
import frc.robot.utils.CommandLogger;
import java.util.List;
import java.util.function.Function;

/**
 * BLine-based autonomous routines. Composes paths from {@link BLinePaths} into full auto commands
 * and registers them in the auto chooser.
 */
public class BLineAutos {

  private final FollowPath.Builder pathBuilder;
  private final SuperStructure superStructure;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShotCalculator hubShotCalculator;
  private final ShotCalculator passCalculator;

  public record BLineAuto(String name, Command auto) {}

  /**
   * @param drivetrain the swerve drivetrain subsystem
   * @param superStructure the superstructure subsystem
   * @param hubShotCalculator calculator for hub shots
   * @param passCalculator calculator for pass shots
   */
  public BLineAutos(
      CommandSwerveDrivetrain drivetrain,
      SuperStructure superStructure,
      ShotCalculator hubShotCalculator,
      ShotCalculator passCalculator) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;
    this.hubShotCalculator = hubShotCalculator;
    this.passCalculator = passCalculator;
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

  /** The full "shoot at hub" command: 4.5s deadline with auto-cycle shooting + face-angle. */
  private Command shootAtHub() {
    return loggedCommand(
        Commands.waitSeconds(4.5)
            .deadlineFor(
                superStructure
                    .setStateCommand(SuperWantedStates.AUTO_CYCLE_SHOOTING)
                    .alongWith(
                        drivetrain.faceAngleWhileDrivingCommand(
                            () -> 0,
                            () -> 0,
                            () -> {
                              if (superStructure.getCurrentSuperState()
                                  == SuperInternalStates.PASSING) {
                                return passCalculator.calculateShot().targetHeading();
                              }
                              return hubShotCalculator.calculateShot().targetHeading();
                            })))
            .alongWith(superStructure.setIntakeStateCommand(IntakeWantedStates.AGITATING))
            .andThen(superStructure.setStateCommand(SuperWantedStates.DEFAULT)),
        "shoot at hub");
  }

  /** Shoot without timer — runs until interrupted. */
  private Command shootWithoutTimer() {
    return loggedCommand(
        superStructure
            .setStateCommand(SuperWantedStates.SHOOT_AT_HUB)
            .alongWith(
                drivetrain.faceAngleWhileDrivingCommand(
                    () -> 0, () -> 0, () -> hubShotCalculator.calculateShot().targetHeading()))
            .finallyDo(() -> superStructure.setWantedSuperState(SuperWantedStates.DEFAULT)),
        "shoot without timer");
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

  private Command buildTwoSwipeAuto(String autoName) {
    return pathWithImmediateIntake(new Path(autoName + " First Swipe"))
        .andThen(shootAtHub())
        .andThen(pathWithImmediateIntake(new Path(autoName + " Second Swipe")))
        .andThen(shootAtHub());
  }

  private BLineAuto buildAutoOrNone(String autoName, Function<String, Command> commandFactory) {
    try {
      return new BLineAuto(autoName, commandFactory.apply(autoName));
    } catch (Exception ignored) {
      return new BLineAuto("!!!NO PATH!!! " + autoName, Commands.none());
    }
  }

  /** Builds all BLine auto routines using generated path variants. */
  private List<BLineAuto> buildAutos() {
    return List.of(
        buildAutoOrNone("Blue Right Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("MIRRORED Blue Left Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("FLIPPED Red Right Aggressive", this::buildTwoSwipeAuto),
        buildAutoOrNone("FLIPPED MIRRORED Red Left Aggressive", this::buildTwoSwipeAuto));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Auto chooser registration
  // ─────────────────────────────────────────────────────────────────────────────

  /**
   * Registers all BLine auto routines into the given chooser with a [BLine] prefix so they appear
   * alongside PathPlanner autos for A/B testing.
   *
   * @param chooser the auto chooser to add options to
   */
  public void registerAutos(SendableChooser<Command> chooser) {
    for (BLineAuto auto : buildAutos()) {
      chooser.addOption("[BLine] " + auto.name, auto.auto);
    }
  }
}
