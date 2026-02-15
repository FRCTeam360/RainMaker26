// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.Vision;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
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

  // Contructor
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
    return intake.setVelocityCommand(4500.0);
  }

  public Command basicShootCmd() {
    return flywheel.setDutyCycleCommand(0.75);
  }

  public Command shootWithRPM(double rpm) {
    return flywheel.setVelocityCommand(rpm);
  }

  public Command shootWithSpinUp(DoubleSupplier rpmSupplier, DoubleSupplier positionSupplier) {
    return hood.setPositionCmd(positionSupplier.getAsDouble())
        .alongWith(flywheel.setVelocityCommand(rpmSupplier.getAsDouble()))
        .alongWith(
            Commands.waitUntil(
                    () ->
                        flywheel.atSetpoint(rpmSupplier.getAsDouble(), 100.0)
                            && hood.atSetpoint(positionSupplier.getAsDouble()))
                .andThen(
                    flyWheelKicker
                        .setVelocityCommand(4500.0)
                        .alongWith(indexer.setDutyCycleCommand(0.4))));
  }

  public Command setFlywheelKickerDutyCycle(double value) {
    return flyWheelKicker.setDutyCycleCommand(value);
  }

  // public Command runHopperAndKicker() {
  // return
  // flyWheelKicker.setVelocityCommand(5000.0).alongWith(indexer.setDutyCycleCommand(0.3));
  // }

  public Command fieldOrientedDriveWithShotCalculator(CommandXboxController controller) {
    return drivetrain
        .fieldOrientedDrive(controller)
        .alongWith(
            Commands.run(
                () -> {
                  shotCalculator.calculateShot();
                }));
  }

  public Command shootWithShotCalculator() {
    return shootWithSpinUp(
        () -> shotCalculator.calculateShot().flywheelSpeed(),
        () -> shotCalculator.calculateShot().hoodAngle());
  }

  public Command setHoodPosition(double position) {
    return hood.setPositionCmd(position);
  }
}
