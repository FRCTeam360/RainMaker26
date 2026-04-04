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

  /** MASTER Red Right Middle Standard — the baseline 2-cycle auto. */
  public Command masterRedRightMiddleStandard() {
    return Commands.sequence(
        Commands.waitSeconds(1.0),
        defaultCmd(),
        pathWithIntake(BLinePaths.noStopRightRedMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.noStopRightRedMiddle2()),
        shootAtHub());
  }

  /** MASTER Red Right Middle Hawk — aggressive 2-cycle. */
  public Command masterRedRightMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.rightRedMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.rightRedMiddleHawk2()),
        shootAtHub());
  }

  /** MASTER Red Right Middle Hook — standard swipe 2-cycle. */
  public Command masterRedRightMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.standardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.standardSwipe2()),
        shootAtHub());
  }

  /** MASTER Red Right Depot — depot-side auto with shoot-without-timer. */
  public Command masterRedRightDepot() {
    return Commands.sequence(
        defaultCmd(),
        followPath(BLinePaths.depot1()),
        Commands.waitSeconds(3.0).deadlineFor(shootWithoutTimer()),
        pathWithImmediateIntake(BLinePaths.depot2()),
        followPath(BLinePaths.depot3()),
        Commands.waitSeconds(5.0).deadlineFor(shootWithoutTimer()));
  }

  /** MASTER Red Right Trench Middle — complex multi-path auto. */
  public Command masterRedRightTrenchMiddle() {
    return Commands.sequence(
        defaultCmd(),
        followPath(BLinePaths.redMiddle1())
            .deadlineFor(Commands.waitSeconds(0.1).andThen(basicIntake()).alongWith(defaultCmd())),
        pathWithImmediateIntake(BLinePaths.redMiddle2()),
        pathWithImmediateIntake(BLinePaths.redMiddle3()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.redMiddle5()),
        pathWithImmediateIntake(BLinePaths.redMiddle6()),
        pathWithImmediateIntake(BLinePaths.redMiddle7()),
        shootAtHub());
  }

  // ── MIRRORED (Red Left) auto routines ──

  public Command mirroredRedLeftMiddleStandard() {
    return Commands.sequence(
        Commands.waitSeconds(1.0),
        defaultCmd(),
        pathWithIntake(BLinePaths.mirroredNoStopLeftRedMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.mirroredNoStopLeftRedMiddle2()),
        shootAtHub());
  }

  public Command mirroredRedLeftMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.mirroredLeftRedMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.mirroredLeftRedMiddleHawk2()),
        shootAtHub());
  }

  public Command mirroredRedLeftMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.mirroredStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.mirroredStandardSwipe2()),
        shootAtHub());
  }

  // ── FLIPPED (Blue Right) auto routines ──

  public Command flippedBlueRightMiddleStandard() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedNoStopRightBlueMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedNoStopRightBlueMiddle2()),
        shootAtHub());
  }

  public Command flippedBlueRightMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedRightBlueMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedRightBlueMiddleHawk2()),
        shootAtHub());
  }

  public Command flippedBlueRightMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedStandardSwipe2()),
        shootAtHub());
  }

  // ── FLIPPED MIRRORED (Blue Left) auto routines ──

  public Command flippedMirroredBlueLeftMiddleStandard() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedMirroredNoStopLeftBlueMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedMirroredNoStopLeftBlueMiddle2()),
        shootAtHub());
  }

  public Command flippedMirroredBlueLeftMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedMirroredLeftBlueMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedMirroredLeftBlueMiddleHawk2()),
        shootAtHub());
  }

  public Command flippedMirroredBlueLeftMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(BLinePaths.flippedMirroredStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(BLinePaths.flippedMirroredStandardSwipe2()),
        shootAtHub());
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
    // MASTER (Red Right)
    chooser.addOption("[BLine] MASTER Red Right Middle Standard", masterRedRightMiddleStandard());
    chooser.addOption("[BLine] MASTER Red Right Middle Hawk", masterRedRightMiddleHawk());
    chooser.addOption("[BLine] MASTER Red Right Middle Hook", masterRedRightMiddleHook());
    chooser.addOption("[BLine] MASTER Red Right Depot", masterRedRightDepot());
    chooser.addOption("[BLine] MASTER Red Right Trench Middle", masterRedRightTrenchMiddle());

    // MIRRORED (Red Left)
    chooser.addOption("[BLine] MIRRORED Red Left Middle Standard", mirroredRedLeftMiddleStandard());
    chooser.addOption("[BLine] MIRRORED Red Left Middle Hawk", mirroredRedLeftMiddleHawk());
    chooser.addOption("[BLine] MIRRORED Red Left Middle Hook", mirroredRedLeftMiddleHook());

    // FLIPPED (Blue Right)
    chooser.addOption(
        "[BLine] FLIPPED Blue Right Middle Standard", flippedBlueRightMiddleStandard());
    chooser.addOption("[BLine] FLIPPED Blue Right Middle Hawk", flippedBlueRightMiddleHawk());
    chooser.addOption("[BLine] FLIPPED Blue Right Middle Hook", flippedBlueRightMiddleHook());

    // FLIPPED MIRRORED (Blue Left)
    chooser.addOption(
        "[BLine] FLIPPED MIRRORED Blue Left Middle Standard",
        flippedMirroredBlueLeftMiddleStandard());
    chooser.addOption(
        "[BLine] FLIPPED MIRRORED Blue Left Middle Hawk", flippedMirroredBlueLeftMiddleHawk());
    chooser.addOption(
        "[BLine] FLIPPED MIRRORED Blue Left Middle Hook", flippedMirroredBlueLeftMiddleHook());

    // GUI-designed paths (single path per auto for testing)
    chooser.addOption("[BLine] Drive Test", followPath(new Path("drive test")));
    chooser.addOption(
        "[BLine] Blue Right Aggressive",
        pathWithIntake(new Path("Blue Right Aggressive first swipe"))
            .andThen(shootAtHub())
            .andThen(pathWithImmediateIntake(new Path("Blue Right Aggressive Second Swipe")))
            .andThen(shootAtHub()));

    chooser.addOption(
        "[BLine] Red Right Aggressive Swipe",
        pathWithIntake(flippedPath("Blue Right Aggressive first swipe"))
            .andThen(shootAtHub())
            .andThen(pathWithImmediateIntake(flippedPath("Blue Right Aggressive Second Swipe")))
            .andThen(shootAtHub()));
  }
}
