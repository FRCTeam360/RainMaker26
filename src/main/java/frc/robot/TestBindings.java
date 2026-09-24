// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ControlState;
import frc.robot.subsystems.HopperRoller.HopperRoller;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.IntakePivot.IntakePivot;
import frc.robot.subsystems.Intake.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.FlywheelKicker.FlywheelKicker;
import frc.robot.subsystems.Shooter.Hood.Hood;
import java.util.function.BooleanSupplier;

/**
 * Bindings and commands for independent (test) mode, run from a dedicated test controller. All
 * bindings are gated on the superstructure being in {@link ControlState#INDEPENDENT} so they cannot
 * interfere with competition (superstructure) mode. Has no competition relevance.
 */
public class TestBindings {

  private static final int TEST_CONTROLLER_PORT = 5;

  private final CommandXboxController testCont = new CommandXboxController(TEST_CONTROLLER_PORT);
  private final BooleanSupplier isIndependentMode;

  private final IntakeRoller intakeRoller;
  private final Indexer indexer;
  private final Hood hood;
  private final FlywheelKicker flywheelKicker;
  private final IntakePivot intakePivot;
  private final HopperRoller hopperRoller;
  private final Flywheel flywheel;

  /**
   * Creates the test bindings holder.
   *
   * @param isIndependentMode supplier that is true while the superstructure is in {@link
   *     ControlState#INDEPENDENT}
   * @param intakeRoller intake roller subsystem
   * @param indexer indexer subsystem
   * @param hood hood subsystem
   * @param flywheelKicker flywheel kicker subsystem
   * @param intakePivot intake pivot subsystem
   * @param hopperRoller hopper roller subsystem
   * @param flywheel flywheel subsystem
   */
  public TestBindings(
      BooleanSupplier isIndependentMode,
      IntakeRoller intakeRoller,
      Indexer indexer,
      Hood hood,
      FlywheelKicker flywheelKicker,
      IntakePivot intakePivot,
      HopperRoller hopperRoller,
      Flywheel flywheel) {
    this.isIndependentMode = isIndependentMode;
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.hood = hood;
    this.flywheelKicker = flywheelKicker;
    this.intakePivot = intakePivot;
    this.hopperRoller = hopperRoller;
    this.flywheel = flywheel;
  }

  /** Configures bindings that are active only in independent (test) mode. */
  public void configure() {
    testCont.leftBumper().and(isIndependentMode).whileTrue(intakeRoller.setVelocityCommand(3000.0));
    testCont.rightBumper().and(isIndependentMode).whileTrue(intakeRoller.setDutyCycleCommand(-0.6));
    testCont.rightTrigger().and(isIndependentMode).whileTrue(indexer.setDutyCycleCommand(0.2));
    testCont.leftTrigger().and(isIndependentMode).whileTrue(indexer.setDutyCycleCommand(-0.2));

    // testCont.a().and(isIndependentMode).whileTrue(indexer.setDutyCycleCommand(0.5));

    // hood bindings
    testCont.pov(0).and(isIndependentMode).onTrue(hood.setPositionCommand(40.0));
    testCont.pov(180).and(isIndependentMode).onTrue(hood.setPositionCommand(0.0));

    testCont.pov(90).and(isIndependentMode).whileTrue(flywheelKicker.setDutyCycleCommand(0.2));
    testCont.pov(270).and(isIndependentMode).whileTrue(flywheelKicker.setDutyCycleCommand(-0.2));

    testCont.a().and(isIndependentMode).onTrue(intakePivot.setPositionCommand(() -> 96.0));
    testCont.y().and(isIndependentMode).onTrue(intakePivot.setPositionCommand(() -> 0.0));

    testCont.x().and(isIndependentMode).whileTrue(hopperRoller.setDutyCycleCommand(0.2));
    testCont.b().and(isIndependentMode).whileTrue(hopperRoller.setDutyCycleCommand(-0.2));

    testCont.start().and(isIndependentMode).whileTrue(flywheel.setDutyCycleCommand(0.2));
    testCont.back().and(isIndependentMode).onTrue(runSystemsTest());
  }

  /**
   * Builds the automated systems test command, which runs each mechanism briefly in sequence.
   *
   * @return the sequential systems test command
   */
  private Command runSystemsTest() {
    return Commands.waitSeconds(0.1)
        .andThen(Commands.waitSeconds(1.0).deadlineFor(intakePivot.setPositionCommand(() -> 96.0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(intakeRoller.setVelocityCommand(3000.0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(intakeRoller.setDutyCycleCommand(-0.6)))
        .andThen(Commands.waitSeconds(0.1).deadlineFor(intakeRoller.setDutyCycleCommand(0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(indexer.setDutyCycleCommand(0.2)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(indexer.setDutyCycleCommand(-0.2)))
        .andThen(Commands.waitSeconds(0.1).deadlineFor(indexer.setDutyCycleCommand(0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(hood.setPositionCommand(40.0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(hood.setPositionCommand(0.0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(flywheelKicker.setDutyCycleCommand(0.2)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(flywheelKicker.setDutyCycleCommand(-0.2)))
        .andThen(Commands.waitSeconds(0.1).deadlineFor(flywheelKicker.setDutyCycleCommand(0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(hopperRoller.setDutyCycleCommand(0.2)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(hopperRoller.setDutyCycleCommand(-0.2)))
        .andThen(Commands.waitSeconds(0.1).deadlineFor(hopperRoller.setDutyCycleCommand(0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(flywheel.setDutyCycleCommand(0.2)))
        .andThen(Commands.waitSeconds(0.1).deadlineFor(flywheel.setDutyCycleCommand(0)))
        .andThen(Commands.waitSeconds(1.0).deadlineFor(intakePivot.setPositionCommand(() -> 0.0)));
  }
}
