package frc.robot.utils;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandLogger {
    /**
     * Logs when the command starts and ends, labelling it with the given name
     *
     * @param command     the command to log
     * @param commandName the unique name of the command
     * @return The provided command with appended logging when the command starts
     *         and ends
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
