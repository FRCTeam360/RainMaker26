// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command basicIntakeCmd() {
    final double INTAKE_VELOCITY_RPM = 4500.0;
    return intake.setVelocityCommand(INTAKE_VELOCITY_RPM);
  }

  public Command basicShootCmd() {
    return flywheel.setDutyCycleCommand(0.75);
  }

  public Command shootWithRPM(double rpm) {
    return flywheel.setVelocityCommand(rpm);
  }

  public Command shootWithSpinUp(DoubleSupplier rpmSupplier, DoubleSupplier positionSupplier) {
    final double FLYWHEEL_TOLERANCE_RPM = 100.0;
    final double KICKER_FEED_VELOCITY_RPM = 4500.0;
    final double INDEXER_FEED_DUTY_CYCLE = 0.4;
    return hood.setPositionCmd(positionSupplier)
        .alongWith(flywheel.setVelocityCommand(rpmSupplier))
        .alongWith(
            Commands.waitUntil(
                    () ->
                        flywheel.atSetpoint(rpmSupplier, FLYWHEEL_TOLERANCE_RPM)
                            && hood.atSetpoint(positionSupplier))
                .andThen(
                    flyWheelKicker
                        .setVelocityCommand(KICKER_FEED_VELOCITY_RPM)
                        .alongWith(indexer.setDutyCycleCommand(INDEXER_FEED_DUTY_CYCLE))));
  }

  public Command setFlywheelKickerDutyCycle(double value) {
    return flyWheelKicker.setDutyCycleCommand(value);
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
