package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public record NamedAutoWithPose(String name, Command auto, Pose2d startingPose) {
  public NamedAuto toNamedAuto() {
    return new NamedAuto(name, auto);
  }
}
