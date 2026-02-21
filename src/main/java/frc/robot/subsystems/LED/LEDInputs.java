package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

// Check for LED temperature (color; warm vs cold (yellow vs white)) and Current (electrical
// current)
@AutoLog
public class LEDInputs {
  public double current;
  public double temperature;
}
