package frc.robot.utils;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;

public class ControllerHelper {
  public static final double DEADBAND = 0.05;

  /**
   * Modifies the axis directly from a controller so that it is correct, with a deadband, scaled,
   * inverted, and clipped.
   *
   * @param value Raw value read from controller axis to be modified.
   * @param scale The amount you want to scale the axis by.
   * @return clipped, scaled, and inverted axis value to use.
   */
  public static double modifyAxisCubic(double value, double scale) {
    double clippedValue = MathUtil.applyDeadband(value, DEADBAND);

    // Cube the clipped value (preserving the sign) and return it.
    return scale * Math.copySign((clippedValue * clippedValue * clippedValue), value);
  }

  public static double modifyAxisCubed(double value, double scale) {
    double clippedValue = MathUtil.applyDeadband(value, DEADBAND);

    // Square the clipped value (preserving the sign) and return it.
    return scale * Math.copySign((clippedValue * clippedValue), value);
  }

  /**
   * Create command to enable rumble feedback on XBox controller when a condition is true with the
   * given rumble type and value.
   *
   * @param controller XBox controller to rumble.
   * @param condition Condition that triggers rumble.
   * @param rumbleType The type of rumble to do.
   * @param value The strength of the rumble.
   * @return Command that triggers rumble feedback on controller when condition is true with the
   *     given rumble type and value.
   */
  public static Command rumble(
      CommandXboxController controller,
      BooleanSupplier condition,
      GenericHID.RumbleType rumbleType,
      double value) {
    return runEnd(
        () -> controller.getHID().setRumble(rumbleType, condition.getAsBoolean() ? value : 0.0),
        () -> controller.getHID().setRumble(rumbleType, 0.0));
  }

  /**
   * Create command to enable rumble feedback on XBox controller with the given rumble type and
   * value.
   *
   * @param controller XBox controller to rumble.
   * @param rumbleType The type of rumble to do.
   * @param value The strength of the rumble.
   * @return Command that triggers rumble feedback on controller with the given rumble type and
   *     value.
   */
  public static Command rumble(
      CommandXboxController controller, GenericHID.RumbleType rumbleType, double value) {
    return rumble(controller, () -> true, rumbleType, value);
  }

  /**
   * Create command to enable rumble feedback on XBox controller when a condition is true.
   *
   * @param controller XBox controller to rumble.
   * @param condition Condition that triggers rumble.
   * @return Command that triggers rumble feedback on controller when condition is true.
   */
  public static Command rumble(CommandXboxController controller, BooleanSupplier condition) {
    return rumble(controller, condition, GenericHID.RumbleType.kBothRumble, 1.0);
  }

  /**
   * Create command to enable rumble feedback on XBox controller.
   *
   * @param controller XBox controller to rumble
   * @return Command that triggers rumble feedback on controller.
   */
  public static Command rumble(CommandXboxController controller) {
    return rumble(controller, () -> true, GenericHID.RumbleType.kBothRumble, 1.0);
  }
}
