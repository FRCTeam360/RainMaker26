package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;

public class LedCandleIo implements LedIO {
  private final CANdle candle;

  private final SolidColor colorRequest = new SolidColor(0, 399);

  public LedCandleIo(CANdle candle) {
    this.candle = candle;
    CANdleConfiguration config = new CANdleConfiguration();

    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LedInputs inputs) {
    inputs.current = candle.getOutputCurrent().getValueAsDouble();
    inputs.temperature = candle.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setColor(Color color) {
    candle.setControl(colorRequest.withColor(new RGBWColor(color)));
  }

  @Override
  public void setAnimation(ControlRequest animation) {
    candle.setControl(animation);
  }
}
