// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;
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
  private final Climber climber;

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
      Climber climber) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.flyWheelKicker = flyWheelKicker;
    this.hood = hood;
    this.indexer = indexer;
    this.intakePivot = intakePivot;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.climber = climber;
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

  public Command setClimberDutyCycleCmd(double dutyCycle) {
    return climber.runEnd(() -> climber.setDutyCycle(dutyCycle), () -> climber.setDutyCycle(0));
  }

  public Command setClimberDutyCycleCmd(DoubleSupplier dutyCycle) {
    return climber.runEnd(
        () -> climber.setDutyCycle(dutyCycle.getAsDouble()), () -> climber.setDutyCycle(0));
  }

  public Command setClimberPositionCmd(double position) {
    return climber.runEnd(() -> climber.setPosition(position), () -> climber.setPosition(position));
  }
}
