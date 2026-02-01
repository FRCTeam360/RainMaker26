// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/** CommandLogger is a utility class that logs the start, running, and end of a command. */
public class CommandLogger {
  /**
   * Logs when the command starts and ends, labelling it with the given name
   *
   * @param command the command to log
   * @param commandName the unique name of the command
   * @return The provided command with appended logging when the command starts and ends
   */
  public static Command logCommand(Command command, String commandName) {
    return command
        .beforeStarting(() -> logCommand(commandName, true))
        .finallyDo(() -> logCommand(commandName, false));
  }

  private static void logCommand(String commandName, boolean isRunning) {
    Logger.recordOutput("Command Running: " + commandName, isRunning);
  }
}
