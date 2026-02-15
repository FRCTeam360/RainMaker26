// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Factory for creating composite commands that coordinate multiple subsystems. Contains the core
 * intake, shoot, and drive commands used in teleop and autonomous.
 */
public class CommandFactory {
  private static final double MAX_VEL = 2.0;
  private static final double MAX_ACCEL = 1.0;

  private final Intake intake;
  private final Flywheel flywheel;
  private final FlywheelKicker flyWheelKicker;
  private final Hood hood;
  private final Indexer indexer;
  private final IntakePivot intakePivot;
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShotCalculator shotCalculator;

  // Constructor
  public CommandFactory(
      Intake intake,
      Flywheel flywheel,
      FlywheelKicker flyWheelKicker,
      Hood hood,
      Indexer indexer,
      IntakePivot intakePivot,
      Vision vision,
      CommandSwerveDrivetrain drivetrain,
      ShotCalculator shotCalculator) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.flyWheelKicker = flyWheelKicker;
    this.hood = hood;
    this.indexer = indexer;
    this.intakePivot = intakePivot;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.shotCalculator = shotCalculator;
  }

  public static Command driveToPose(CommandSwerveDrivetrain drive, Pose2d targetPose) {
    ProfiledPIDController xController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL));
    ProfiledPIDController yController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL));
    ProfiledPIDController thetaController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            () -> {
              Pose2d currentPose = drive.getPose();

              double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
              double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
              double thetaSpeed =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians());

              drive.drive(xSpeed, ySpeed, thetaSpeed);
            })
        .until(() -> xController.atGoal() && yController.atGoal() && thetaController.atGoal())
        .withName("DriveToClimbPose");
  }

  public Command basicIntakeCmd() {
    return intake.setVelocityCommand(4500.0).alongWith(indexer.setDutyCycleCommand(0.4));
  }

  public Command basicShootCmd() {
    return flywheel.setDutyCycleCommand(0.75);
  }

  public Command shootWithRPM(double rpm) {
    return flywheel.setRPMCommand(rpm);
  }

  public Command shootWithSpinUp(double rpm, double position) {
    return hood.setPositionCmd(position)
        .alongWith(flywheel.setRPMCommand(rpm))
        .alongWith(
            Commands.waitUntil(() -> flywheel.atSetpoint(rpm, 100.0) && hood.atSetpoint(position))
                .andThen(
                    flyWheelKicker.setVelocityCommand(4500.0).alongWith(this.basicIntakeCmd())));
  }

  public Command setFlywheelKickerDutyCycle(double value) {
    return flyWheelKicker.setDutyCycleCommand(value);
  }

  public Command setHoodPosition(double position) {
    return hood.setPositionCmd(position);
  }

  /**
   * Creates a command that aims at the ShotCalculator target (either custom dashboard target or
   * hub) and auto-fires when ready. The driver retains translational control while the heading
   * auto-tracks the target.
   *
   * <p>Sequence:
   *
   * <ol>
   *   <li>Calculate shot parameters (hood angle, flywheel RPM, heading) from distance to target
   *   <li>Rotate drivetrain to face target while driver controls translation
   *   <li>Spin up flywheel and adjust hood simultaneously
   *   <li>When flywheel at speed, hood at position, and heading aligned: auto-fire via kicker +
   *       intake
   * </ol>
   *
   * @param driveCont the driver Xbox controller for translation input
   * @return the composite shoot-at-target command
   */
  public Command shootAtTargetCmd(CommandXboxController driveCont) {
    return Commands.parallel(
            // Continuously recalculate shot params and aim drivetrain at the target
            drivetrain.facePointWhileDriving(driveCont, () -> shotCalculator.getTargetPosition()),

            // Continuously run the ShotCalculator so params stay fresh
            Commands.run(
                () -> {
                  shotCalculator.clearShootingParams();
                  shotCalculator.calculateShot();
                }),

            // Set hood + flywheel from ShotCalculator, then auto-fire when aligned
            Commands.sequence(
                // Spin up flywheel and set hood based on calculated params
                hood.run(
                        () -> {
                          ShotCalculator.ShootingParams params = shotCalculator.calculateShot();
                          hood.setPosition(params.hoodAngle());
                          flywheel.setRPM(params.flywheelSpeed());
                        })
                    .until(
                        () -> {
                          ShotCalculator.ShootingParams params = shotCalculator.calculateShot();
                          Rotation2d currentHeading = drivetrain.getRotation2d();
                          double headingError =
                              Math.abs(
                                  currentHeading.getRadians() - params.targetAngle().getRadians());
                          // Normalize heading error to [-pi, pi]
                          headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                          boolean flywheelReady =
                              flywheel.atSetpoint(params.flywheelSpeed(), 100.0);
                          boolean hoodReady = hood.atSetpoint(params.hoodAngle());
                          boolean headingAligned =
                              Math.abs(headingError) < ShooterConstants.HEADING_TOLERANCE_RAD;

                          // Log readiness state for debugging in AdvantageScope
                          Logger.recordOutput(
                              "ShootAtTarget/headingErrorDeg", Math.toDegrees(headingError));
                          Logger.recordOutput("ShootAtTarget/flywheelReady", flywheelReady);
                          Logger.recordOutput("ShootAtTarget/hoodReady", hoodReady);
                          Logger.recordOutput("ShootAtTarget/headingAligned", headingAligned);
                          Logger.recordOutput(
                              "ShootAtTarget/allReady",
                              flywheelReady && hoodReady && headingAligned);

                          return flywheelReady && hoodReady && headingAligned;
                        }),
                // Auto-fire: engage kicker + intake to feed the ball
                flyWheelKicker.setVelocityCommand(4500.0).alongWith(basicIntakeCmd())))
        .finallyDo(
            () -> {
              shotCalculator.clearShootingParams();
            })
        .withName("ShootAtTarget");
  }
}
