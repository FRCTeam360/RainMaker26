// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood.Hood;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Command to tune the hood position using a LoggedNetworkNumber.
 *
 * <p>This command continuously reads from a NetworkTables tunable value and sets the hood position
 * accordingly. Useful for testing and tuning hood angles during development.
 */
public class HoodTuneCommand extends Command {
  private final Hood hood;
  private final LoggedNetworkNumber tunablePosition =
      new LoggedNetworkNumber("/Tuning/Hood/TunablePosition", 0.0);

  /**
   * Creates a new HoodTuneCommand.
   *
   * @param hood The hood subsystem to control
   */
  public HoodTuneCommand(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hood.setPosition(tunablePosition.get());
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
