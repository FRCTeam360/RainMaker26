package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeStateMachine.IntakeWantedStates;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperInternalStates;
import frc.robot.subsystems.SuperStructure.SuperWantedStates;
import frc.robot.utils.CommandLogger;

/**
 * BLine-based autonomous routines. Builds all 14 auto variants using programmatic path definitions
 * converted from PathPlanner waypoint coordinates.
 *
 * <p>Paths are defined once for the MASTER (Red Right) variants, then transformed for MIRRORED (Red
 * Left), FLIPPED (Blue Right), and FLIPPED MIRRORED (Blue Left) using coordinate transformations.
 */
public class BLineAutos {

  private static final double FIELD_LENGTH_METERS = 16.54;
  private static final double FIELD_WIDTH_METERS = 8.07;

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

  /** Mirrors Y coordinate about field centerline (Red Right → Red Left). */
  private static Translation2d mirrorY(Translation2d t) {
    return new Translation2d(t.getX(), FIELD_WIDTH_METERS - t.getY());
  }

  /** Mirrors a rotation about the X axis (negates angle). */
  private static Rotation2d mirrorRotation(Rotation2d r) {
    return r.unaryMinus();
  }

  /** Flips X coordinate about field centerline (Red → Blue). */
  private static Translation2d flipX(Translation2d t) {
    return new Translation2d(FIELD_LENGTH_METERS - t.getX(), t.getY());
  }

  /** Flips a rotation by 180 degrees (Red → Blue). */
  private static Rotation2d flipRotation(Rotation2d r) {
    return r.rotateBy(Rotation2d.k180deg);
  }

  /** Applies both flip and mirror (Red Right → Blue Left). */
  private static Translation2d flipAndMirror(Translation2d t) {
    return mirrorY(flipX(t));
  }

  private static Rotation2d flipAndMirrorRotation(Rotation2d r) {
    return mirrorRotation(flipRotation(r));
  }

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
                            }))
                    .alongWith(
                        Commands.waitSeconds(2.25)
                            .andThen(
                                superStructure.setIntakeStateCommand(
                                    IntakeWantedStates.AGITATING))))
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

  // ─────────────────────────────────────────────────────────────────────────────
  // Path definitions — MASTER (Red Right) coordinates
  // ─────────────────────────────────────────────────────────────────────────────

  // Rotation constants (degrees → Rotation2d) used across multiple paths
  private static final Rotation2d ROT_0 = Rotation2d.fromDegrees(0);
  private static final Rotation2d ROT_180 = Rotation2d.fromDegrees(180);
  private static final Rotation2d ROT_90 = Rotation2d.fromDegrees(90);
  private static final Rotation2d ROT_NEG_90 = Rotation2d.fromDegrees(-90);

  // ── No Stop Right Red Middle paths ──

  private static Path noStopRightRedMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 7.485), ROT_180),
        new TranslationTarget(9.089, 7.290),
        new RotationTarget(Rotation2d.fromDegrees(-117.273), 0.5),
        new TranslationTarget(9.089, 5.302),
        new RotationTarget(Rotation2d.fromDegrees(-78.434), 0.5),
        new TranslationTarget(10.638, 5.536),
        new RotationTarget(Rotation2d.fromDegrees(-9.644), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)));
  }

  private static Path noStopRightRedMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)),
        new TranslationTarget(14.623, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-173.062), 0.5),
        new TranslationTarget(13.444, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-169.236), 0.5),
        new TranslationTarget(10.385, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-85.796), 0.5),
        new TranslationTarget(10.570, 5.107),
        new RotationTarget(Rotation2d.fromDegrees(-87.075), 0.5),
        new TranslationTarget(9.995, 4.396),
        new RotationTarget(Rotation2d.fromDegrees(164.633), 0.5),
        new TranslationTarget(8.855, 5.273),
        new RotationTarget(Rotation2d.fromDegrees(10.025), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(21.501)));
  }

  // ── Right Red Middle Hawk paths ──

  private static Path rightRedMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 7.485), ROT_180),
        new TranslationTarget(8.670, 7.358),
        new RotationTarget(Rotation2d.fromDegrees(-95.919), 0.5),
        new TranslationTarget(8.670, 4.464),
        new RotationTarget(Rotation2d.fromDegrees(-64.529), 0.5),
        new TranslationTarget(10.404, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-24.079), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)));
  }

  private static Path rightRedMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(23.026)),
        new TranslationTarget(14.623, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-173.062), 0.5),
        new TranslationTarget(13.444, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-172.367), 0.5),
        new TranslationTarget(10.531, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-79.167), 0.5),
        new TranslationTarget(10.726, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-102.987), 0.5),
        new TranslationTarget(8.572, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-179.864), 0.5),
        new TranslationTarget(8.767, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-17.886), 0.5),
        new Waypoint(new Translation2d(13.854, 5.721), Rotation2d.fromDegrees(21.501)));
  }

  // ── Standard Swipe paths (Hook auto) ──

  private static Path standardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(12.096, 7.436), Rotation2d.fromDegrees(179.384)),
        new TranslationTarget(8.774, 5.468),
        new RotationTarget(ROT_NEG_90, 0.5),
        new TranslationTarget(10.301, 5.279),
        new RotationTarget(Rotation2d.fromDegrees(40.0), 0.5),
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(40.764)));
  }

  private static Path standardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(40.764)),
        new TranslationTarget(13.104, 7.373),
        new RotationTarget(ROT_180, 0.5),
        new TranslationTarget(10.774, 7.373),
        new RotationTarget(Rotation2d.fromDegrees(-179.148), 0.5),
        new TranslationTarget(10.569, 3.925),
        new RotationTarget(Rotation2d.fromDegrees(-69.738), 0.5),
        new TranslationTarget(8.947, 3.657),
        new RotationTarget(Rotation2d.fromDegrees(145.0), 0.5),
        new TranslationTarget(9.120, 5.373),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.873, 5.468), Rotation2d.fromDegrees(45.0)));
  }

  // ── Trench Middle paths (Red middle 1-7) ──

  private static Path redMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.090, 7.505), ROT_180),
        new Waypoint(new Translation2d(8.514, 7.202), Rotation2d.fromDegrees(-79.16)));
  }

  private static Path redMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(8.514, 7.202), Rotation2d.fromDegrees(-79.16)),
        new Waypoint(new Translation2d(8.514, 5.058), Rotation2d.fromDegrees(-79.16)));
  }

  private static Path redMiddle3() {
    return new Path(
        new Waypoint(new Translation2d(8.514, 5.058), Rotation2d.fromDegrees(-79.16)),
        new TranslationTarget(10.094, 7.600),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(13.337, 7.348),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.678, 5.604), Rotation2d.fromDegrees(52.058)));
  }

  private static Path redMiddle5() {
    return new Path(
        new Waypoint(new Translation2d(13.678, 5.604), Rotation2d.fromDegrees(52.058)),
        new TranslationTarget(12.889, 7.465),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(10.275, 7.465),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(10.043, 7.046), Rotation2d.fromDegrees(-75.619)));
  }

  private static Path redMiddle6() {
    return new Path(
        new Waypoint(new Translation2d(10.043, 7.046), Rotation2d.fromDegrees(-75.619)),
        new Waypoint(new Translation2d(10.780, 5.258), ROT_NEG_90));
  }

  private static Path redMiddle7() {
    return new Path(
        new Waypoint(new Translation2d(10.780, 5.258), ROT_NEG_90),
        new TranslationTarget(9.790, 7.494),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(12.411, 7.494),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.610, 6.715), Rotation2d.fromDegrees(57.724)));
  }

  // ── Depot paths ──

  private static Path depot1() {
    return new Path(
        new Waypoint(new Translation2d(12.996, 1.969), ROT_NEG_90),
        new Waypoint(new Translation2d(14.185, 1.784), Rotation2d.fromDegrees(-45.785)));
  }

  private static Path depot2() {
    return new Path(
        new Waypoint(new Translation2d(14.185, 1.784), Rotation2d.fromDegrees(-45.785)),
        new TranslationTarget(16.056, 0.585),
        new RotationTarget(ROT_90, 0.5),
        new Waypoint(new Translation2d(16.046, 3.129), ROT_90));
  }

  private static Path depot3() {
    return new Path(
        new Waypoint(new Translation2d(16.046, 3.129), ROT_90),
        new Waypoint(new Translation2d(13.824, 3.129), Rotation2d.fromDegrees(-45.785)));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Path transformations — apply mirror/flip to create variants
  // ─────────────────────────────────────────────────────────────────────────────

  /**
   * Represents a coordinate transformation strategy for generating alliance/side variants from
   * master paths.
   */
  @FunctionalInterface
  private interface CoordTransform {
    Translation2d applyTranslation(Translation2d t);
  }

  @FunctionalInterface
  private interface RotTransform {
    Rotation2d applyRotation(Rotation2d r);
  }

  /**
   * Transforms a master path using coordinate and rotation transforms. This creates new path
   * elements by applying position/rotation mappings to each element.
   */
  private static Path transformPath(Path master, CoordTransform ct, RotTransform rt) {
    // We need to rebuild the path from its elements. Since BLine Path doesn't expose
    // individual elements for iteration, we define transformed variants directly.
    // This method exists as a conceptual placeholder — actual transformed paths are
    // built using the same Waypoint/TranslationTarget constructors with transformed coords.
    throw new UnsupportedOperationException(
        "Use the explicit transformed path factory methods instead");
  }

  // ── MIRRORED (Red Left) path factories ──

  private static Path mirroredNoStopLeftRedMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 0.585), ROT_180),
        new TranslationTarget(9.089, 0.780),
        new RotationTarget(Rotation2d.fromDegrees(117.273), 0.5),
        new TranslationTarget(9.089, 2.768),
        new RotationTarget(Rotation2d.fromDegrees(78.434), 0.5),
        new TranslationTarget(10.638, 2.534),
        new RotationTarget(Rotation2d.fromDegrees(9.644), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)));
  }

  private static Path mirroredNoStopLeftRedMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)),
        new TranslationTarget(14.623, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(173.062), 0.5),
        new TranslationTarget(13.444, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(169.236), 0.5),
        new TranslationTarget(10.385, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(85.796), 0.5),
        new TranslationTarget(10.570, 2.963),
        new RotationTarget(Rotation2d.fromDegrees(87.075), 0.5),
        new TranslationTarget(9.995, 3.674),
        new RotationTarget(Rotation2d.fromDegrees(-164.633), 0.5),
        new TranslationTarget(8.855, 2.797),
        new RotationTarget(Rotation2d.fromDegrees(-10.025), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-21.501)));
  }

  private static Path mirroredLeftRedMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(12.109, 0.585), ROT_180),
        new TranslationTarget(8.670, 0.712),
        new RotationTarget(Rotation2d.fromDegrees(95.919), 0.5),
        new TranslationTarget(8.670, 3.606),
        new RotationTarget(Rotation2d.fromDegrees(64.529), 0.5),
        new TranslationTarget(10.404, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(24.079), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)));
  }

  private static Path mirroredLeftRedMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-23.026)),
        new TranslationTarget(14.623, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(173.062), 0.5),
        new TranslationTarget(13.444, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(172.367), 0.5),
        new TranslationTarget(10.531, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(79.167), 0.5),
        new TranslationTarget(10.726, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(102.987), 0.5),
        new TranslationTarget(8.572, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(179.864), 0.5),
        new TranslationTarget(8.767, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(17.886), 0.5),
        new Waypoint(new Translation2d(13.854, 2.349), Rotation2d.fromDegrees(-21.501)));
  }

  private static Path mirroredStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(12.096, 0.634), Rotation2d.fromDegrees(-179.384)),
        new TranslationTarget(8.774, 2.602),
        new RotationTarget(ROT_90, 0.5),
        new TranslationTarget(10.301, 2.791),
        new RotationTarget(Rotation2d.fromDegrees(-40.0), 0.5),
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-40.764)));
  }

  private static Path mirroredStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-40.764)),
        new TranslationTarget(13.104, 0.697),
        new RotationTarget(ROT_180, 0.5),
        new TranslationTarget(10.774, 0.697),
        new RotationTarget(Rotation2d.fromDegrees(179.148), 0.5),
        new TranslationTarget(10.569, 4.145),
        new RotationTarget(Rotation2d.fromDegrees(69.738), 0.5),
        new TranslationTarget(8.947, 4.413),
        new RotationTarget(Rotation2d.fromDegrees(-145.0), 0.5),
        new TranslationTarget(9.120, 2.697),
        new RotationTarget(ROT_0, 0.5),
        new Waypoint(new Translation2d(13.873, 2.602), Rotation2d.fromDegrees(-45.0)));
  }

  // ── FLIPPED (Blue Right) path factories ──

  private static Path flippedNoStopRightBlueMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 0.585), ROT_0),
        new TranslationTarget(7.451, 0.780),
        new RotationTarget(Rotation2d.fromDegrees(62.727), 0.5),
        new TranslationTarget(7.451, 2.768),
        new RotationTarget(Rotation2d.fromDegrees(101.566), 0.5),
        new TranslationTarget(5.902, 2.534),
        new RotationTarget(Rotation2d.fromDegrees(170.356), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)));
  }

  private static Path flippedNoStopRightBlueMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)),
        new TranslationTarget(1.917, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(6.938), 0.5),
        new TranslationTarget(3.096, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(10.764), 0.5),
        new TranslationTarget(6.155, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(94.204), 0.5),
        new TranslationTarget(5.970, 2.963),
        new RotationTarget(Rotation2d.fromDegrees(92.925), 0.5),
        new TranslationTarget(6.545, 3.674),
        new RotationTarget(Rotation2d.fromDegrees(-15.368), 0.5),
        new TranslationTarget(7.685, 2.797),
        new RotationTarget(Rotation2d.fromDegrees(-169.975), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-158.499)));
  }

  private static Path flippedRightBlueMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 0.585), ROT_0),
        new TranslationTarget(7.870, 0.712),
        new RotationTarget(Rotation2d.fromDegrees(84.081), 0.5),
        new TranslationTarget(7.870, 3.606),
        new RotationTarget(Rotation2d.fromDegrees(115.471), 0.5),
        new TranslationTarget(6.136, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(155.921), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)));
  }

  private static Path flippedRightBlueMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-156.975)),
        new TranslationTarget(1.917, 1.307),
        new RotationTarget(Rotation2d.fromDegrees(6.938), 0.5),
        new TranslationTarget(3.096, 0.576),
        new RotationTarget(Rotation2d.fromDegrees(7.633), 0.5),
        new TranslationTarget(6.009, 0.975),
        new RotationTarget(Rotation2d.fromDegrees(100.834), 0.5),
        new TranslationTarget(5.814, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(77.013), 0.5),
        new TranslationTarget(7.968, 3.782),
        new RotationTarget(Rotation2d.fromDegrees(0.136), 0.5),
        new TranslationTarget(7.773, 2.349),
        new RotationTarget(Rotation2d.fromDegrees(162.114), 0.5),
        new Waypoint(new Translation2d(2.686, 2.349), Rotation2d.fromDegrees(-158.499)));
  }

  private static Path flippedStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(4.444, 0.634), Rotation2d.fromDegrees(-0.616)),
        new TranslationTarget(7.766, 2.602),
        new RotationTarget(ROT_90, 0.5),
        new TranslationTarget(6.239, 2.791),
        new RotationTarget(Rotation2d.fromDegrees(-140.0), 0.5),
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-139.236)));
  }

  private static Path flippedStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-139.236)),
        new TranslationTarget(3.436, 0.697),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(5.766, 0.697),
        new RotationTarget(Rotation2d.fromDegrees(0.852), 0.5),
        new TranslationTarget(5.971, 4.145),
        new RotationTarget(Rotation2d.fromDegrees(110.262), 0.5),
        new TranslationTarget(7.593, 4.413),
        new RotationTarget(Rotation2d.fromDegrees(-35.0), 0.5),
        new TranslationTarget(7.420, 2.697),
        new RotationTarget(ROT_180, 0.5),
        new Waypoint(new Translation2d(2.667, 2.602), Rotation2d.fromDegrees(-135.0)));
  }

  // ── FLIPPED MIRRORED (Blue Left) path factories ──

  private static Path flippedMirroredNoStopLeftBlueMiddle1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 7.485), ROT_0),
        new TranslationTarget(7.451, 7.290),
        new RotationTarget(Rotation2d.fromDegrees(-62.727), 0.5),
        new TranslationTarget(7.451, 5.302),
        new RotationTarget(Rotation2d.fromDegrees(-101.566), 0.5),
        new TranslationTarget(5.902, 5.536),
        new RotationTarget(Rotation2d.fromDegrees(-170.356), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)));
  }

  private static Path flippedMirroredNoStopLeftBlueMiddle2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)),
        new TranslationTarget(1.917, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-6.938), 0.5),
        new TranslationTarget(3.096, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-10.764), 0.5),
        new TranslationTarget(6.155, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-94.204), 0.5),
        new TranslationTarget(5.970, 5.107),
        new RotationTarget(Rotation2d.fromDegrees(-92.925), 0.5),
        new TranslationTarget(6.545, 4.396),
        new RotationTarget(Rotation2d.fromDegrees(15.368), 0.5),
        new TranslationTarget(7.685, 5.273),
        new RotationTarget(Rotation2d.fromDegrees(169.975), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(158.499)));
  }

  private static Path flippedMirroredLeftBlueMiddleHawk1() {
    return new Path(
        new Waypoint(new Translation2d(4.431, 7.485), ROT_0),
        new TranslationTarget(7.870, 7.358),
        new RotationTarget(Rotation2d.fromDegrees(-84.081), 0.5),
        new TranslationTarget(7.870, 4.464),
        new RotationTarget(Rotation2d.fromDegrees(-115.471), 0.5),
        new TranslationTarget(6.136, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-155.921), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)));
  }

  private static Path flippedMirroredLeftBlueMiddleHawk2() {
    return new Path(
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(156.975)),
        new TranslationTarget(1.917, 6.763),
        new RotationTarget(Rotation2d.fromDegrees(-6.938), 0.5),
        new TranslationTarget(3.096, 7.494),
        new RotationTarget(Rotation2d.fromDegrees(-7.633), 0.5),
        new TranslationTarget(6.009, 7.095),
        new RotationTarget(Rotation2d.fromDegrees(-100.834), 0.5),
        new TranslationTarget(5.814, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-77.013), 0.5),
        new TranslationTarget(7.968, 4.288),
        new RotationTarget(Rotation2d.fromDegrees(-0.136), 0.5),
        new TranslationTarget(7.773, 5.721),
        new RotationTarget(Rotation2d.fromDegrees(-162.114), 0.5),
        new Waypoint(new Translation2d(2.686, 5.721), Rotation2d.fromDegrees(158.499)));
  }

  private static Path flippedMirroredStandardSwipe1() {
    return new Path(
        new Waypoint(new Translation2d(4.444, 7.436), Rotation2d.fromDegrees(0.616)),
        new TranslationTarget(7.766, 5.468),
        new RotationTarget(ROT_NEG_90, 0.5),
        new TranslationTarget(6.239, 5.279),
        new RotationTarget(Rotation2d.fromDegrees(140.0), 0.5),
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(139.236)));
  }

  private static Path flippedMirroredStandardSwipe2() {
    return new Path(
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(139.236)),
        new TranslationTarget(3.436, 7.373),
        new RotationTarget(ROT_0, 0.5),
        new TranslationTarget(5.766, 7.373),
        new RotationTarget(Rotation2d.fromDegrees(-0.852), 0.5),
        new TranslationTarget(5.971, 3.925),
        new RotationTarget(Rotation2d.fromDegrees(-110.262), 0.5),
        new TranslationTarget(7.593, 3.657),
        new RotationTarget(Rotation2d.fromDegrees(35.0), 0.5),
        new TranslationTarget(7.420, 5.373),
        new RotationTarget(ROT_180, 0.5),
        new Waypoint(new Translation2d(2.667, 5.468), Rotation2d.fromDegrees(135.0)));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Auto routine compositions
  // ─────────────────────────────────────────────────────────────────────────────

  /** MASTER Red Right Middle Standard — the baseline 2-cycle auto. */
  public Command masterRedRightMiddleStandard() {
    return Commands.sequence(
        Commands.waitSeconds(1.0),
        defaultCmd(),
        pathWithIntake(noStopRightRedMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(noStopRightRedMiddle2()),
        shootAtHub());
  }

  /** MASTER Red Right Middle Hawk — aggressive 2-cycle. */
  public Command masterRedRightMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(rightRedMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(rightRedMiddleHawk2()),
        shootAtHub());
  }

  /** MASTER Red Right Middle Hook — standard swipe 2-cycle. */
  public Command masterRedRightMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(standardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(standardSwipe2()),
        shootAtHub());
  }

  /** MASTER Red Right Depot — depot-side auto with shoot-without-timer. */
  public Command masterRedRightDepot() {
    return Commands.sequence(
        defaultCmd(),
        followPath(depot1()),
        Commands.waitSeconds(3.0).deadlineFor(shootWithoutTimer()),
        pathWithImmediateIntake(depot2()),
        followPath(depot3()),
        Commands.waitSeconds(5.0).deadlineFor(shootWithoutTimer()));
  }

  /** MASTER Red Right Trench Middle — complex multi-path auto. */
  public Command masterRedRightTrenchMiddle() {
    return Commands.sequence(
        defaultCmd(),
        followPath(redMiddle1())
            .deadlineFor(Commands.waitSeconds(0.1).andThen(basicIntake()).alongWith(defaultCmd())),
        pathWithImmediateIntake(redMiddle2()),
        pathWithImmediateIntake(redMiddle3()),
        shootAtHub(),
        pathWithImmediateIntake(redMiddle5()),
        pathWithImmediateIntake(redMiddle6()),
        pathWithImmediateIntake(redMiddle7()),
        shootAtHub());
  }

  // ── MIRRORED (Red Left) auto routines ──

  public Command mirroredRedLeftMiddleStandard() {
    return Commands.sequence(
        Commands.waitSeconds(1.0),
        defaultCmd(),
        pathWithIntake(mirroredNoStopLeftRedMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(mirroredNoStopLeftRedMiddle2()),
        shootAtHub());
  }

  public Command mirroredRedLeftMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(mirroredLeftRedMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(mirroredLeftRedMiddleHawk2()),
        shootAtHub());
  }

  public Command mirroredRedLeftMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(mirroredStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(mirroredStandardSwipe2()),
        shootAtHub());
  }

  // ── FLIPPED (Blue Right) auto routines ──

  public Command flippedBlueRightMiddleStandard() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedNoStopRightBlueMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedNoStopRightBlueMiddle2()),
        shootAtHub());
  }

  public Command flippedBlueRightMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedRightBlueMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedRightBlueMiddleHawk2()),
        shootAtHub());
  }

  public Command flippedBlueRightMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedStandardSwipe2()),
        shootAtHub());
  }

  // ── FLIPPED MIRRORED (Blue Left) auto routines ──

  public Command flippedMirroredBlueLeftMiddleStandard() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedMirroredNoStopLeftBlueMiddle1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedMirroredNoStopLeftBlueMiddle2()),
        shootAtHub());
  }

  public Command flippedMirroredBlueLeftMiddleHawk() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedMirroredLeftBlueMiddleHawk1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedMirroredLeftBlueMiddleHawk2()),
        shootAtHub());
  }

  public Command flippedMirroredBlueLeftMiddleHook() {
    return Commands.sequence(
        defaultCmd(),
        pathWithIntake(flippedMirroredStandardSwipe1()),
        shootAtHub(),
        pathWithImmediateIntake(flippedMirroredStandardSwipe2()),
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
            .andThen(pathWithImmediateIntake(new Path("Blue Right Aggressive Second Swipe"))));
  }
}
