package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;

/** A named autonomous routine pairing a display name with its command. */
public record NamedAuto(String name, Command auto) {}
