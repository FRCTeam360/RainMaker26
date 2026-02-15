// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Command to tune the flywheel RPM using a LoggedNetworkNumber.
 *
 * <p>This command continuously reads from a NetworkTables tunable value and sets the flywheel RPM
 * accordingly. Useful for testing and tuning flywheel speeds during development.
 */
public class FlywheelTuneCommand extends Command {
  private final Flywheel flywheel;
  private final LoggedNetworkNumber tunableRPM =
      new LoggedNetworkNumber("/Tuning/Flywheel/TunableRPM", 0.0);

  /**
   * Creates a new FlywheelTuneCommand.
   *
   * @param flywheel The flywheel subsystem to control
   */
  public FlywheelTuneCommand(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheel.setRPM(tunableRPM.get());
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
