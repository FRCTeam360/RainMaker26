// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Vision.Vision;

/** Add your docs here. */
public class CommandFactory {

  private final Intake intake;
  private final Flywheel flywheel;
  private final FlywheelKicker flyWheelKicker;
  // private final Hood hood;
  private final Indexer indexer;
  // private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;

  // Contructor
  public CommandFactory(
      Intake intake,
      Flywheel flywheel,
      FlywheelKicker flyWheelKicker,
      // Hood hood,
      Indexer indexer,
      // Vision vision,
      CommandSwerveDrivetrain drivetrain) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.flyWheelKicker = flyWheelKicker;
    // this.hood = hood;
    this.indexer = indexer;
    // this.vision = vision;
    this.drivetrain = drivetrain;
  }

  public Command basicIntakeCmd() {
    return intake
        .setDutyCycleCommand(0.65)
        .alongWith(flyWheelKicker.setDutyCycleCommand(1.0))
        .alongWith(indexer.setDutyCycleCommand(0.5));
  }

  public Command basicShootCmd() {
    return flywheel.setDutyCycleCommand(0.75);
  }

  public Command shootWithRPM(double rpm) {
    return flywheel.setRPMCommand(rpm);
  }

  public Command setFlywheelKickerDutyCycle(double value) {
    return flyWheelKicker.setDutyCycleCommand(value);
  }
}
