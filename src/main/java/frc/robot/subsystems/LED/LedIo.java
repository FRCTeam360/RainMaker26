package frc.robot.subsystems.LED;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.wpilibj.util.Color;

public interface LedIo {
  void updateInputs(LEDInputs inputs);

  void setColor(Color color);

  void setAnimation(ControlRequest request);
}
