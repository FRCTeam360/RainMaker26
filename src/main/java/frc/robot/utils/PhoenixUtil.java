package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts a command up to maxAttempts times until it returns OK. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var status = command.get();
      if (status.isOK()) break;
    }
  }
}
