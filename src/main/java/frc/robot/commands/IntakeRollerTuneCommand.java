// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeRoller.IntakeRoller;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Command to tune the intake roller velocity PID using LoggedNetworkNumbers.
 *
 * <p>This command continuously reads from NetworkTables tunable values and sets the intake roller
 * velocity and PID gains accordingly. PID gains are only pushed to the motor controller when a
 * change is detected. Useful for testing and tuning intake speeds during development.
 */
public class IntakeRollerTuneCommand extends Command {
  private final IntakeRoller intakeRoller;

  private final LoggedNetworkNumber tunableRPM =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/TunableRPM", 0.0);
  private final LoggedNetworkNumber tunableKP =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/kP", 0.0001);
  private final LoggedNetworkNumber tunableKI =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/kI", 0.0);
  private final LoggedNetworkNumber tunableKD =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/kD", 0.0);
  private final LoggedNetworkNumber tunableKV =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/kV", 0.009);
  private final LoggedNetworkNumber tunableKS =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/kS", 0.02);

  private double lastKP = 0.0001;
  private double lastKI = 0.0;
  private double lastKD = 0.0;
  private double lastKV = 0.009;
  private double lastKS = 0.02;

  /**
   * Creates a new IntakeRollerTuneCommand.
   *
   * @param intakeRoller The intake roller subsystem to control
   */
  public IntakeRollerTuneCommand(IntakeRoller intakeRoller) {
    this.intakeRoller = intakeRoller;
    addRequirements(intakeRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double kP = tunableKP.get();
    double kI = tunableKI.get();
    double kD = tunableKD.get();
    double kV = tunableKV.get();
    double kS = tunableKS.get();

    if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV || kS != lastKS) {
      intakeRoller.setPID(kP, kI, kD, kV, kS);
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
      lastKV = kV;
      lastKS = kS;
    }

    intakeRoller.setVelocity(tunableRPM.get());
  }

  @Override
  public void end(boolean interrupted) {
    intakeRoller.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
