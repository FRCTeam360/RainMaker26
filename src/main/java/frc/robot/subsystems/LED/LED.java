// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Thank you to FRC3663 for inspiration and reference for this code!!!!!!!

package frc.robot.subsystems.LED;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  
  private final double ANIMATION_SPEED = 0.001;
  private final double LED_BRIGHTNESS = 1.0;
  private final double NUM_LEDS = 200;
  private final int POCKET_SIZE = 3;

  private final LedIo io;
  private final LedInputs inputs = new LEDInputs();
  private CANdle candle;


  private final RgbFadeAnimation startAnimation = new RgbFadeAnimation(8, 399).withFrameRate(ANIMATION_SPEED).withBrightness(LED_BRIGHTNESS);
  private StatusCode currentAnimation;
  private Color currentColor = new Color();
  private Patterns currentPattern = Patterns.SOLID;
  private LarsonAnimation larsonAnimation;
  private RgbFadeAnimation rgbFadeAnimation;
  private StrobeAnimation strobeAnimation;

  public Led(LedIo io) {
    this.io = io;
    this.candle = candle;
    io.setAnimation(startAnimation);
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setColor(Color color) {
    currentColor = color;
    setPattern(currentPattern);
  }

  public void setPattern(Patterns pattern) {
    switch (pattern) {
      case SOLID:
        io.setAnimation(null);
        io.setColor(currentColor);
        currentPattern = Patterns.SOLID;
        break;

      case FADE:
        io.setColor(currentColor);
        rgbFadeAnimation = new RgbFadeAnimation(8, 399).withSlot(2).withFrameRate(ANIMATION_SPEED).withBrightness(LED_BRIGHTNESS);
        io.setAnimation(rgbFadeAnimation);
        currentPattern = Patterns.FADE;
        break;

      case LARSON:
        io.setColor(currentColor);
        rgbFadeAnimation = new LarsonAnimation(8,399).withSlot(3).withFrameRate(ANIMATION_SPEED).withColor(new RGBWColor(currentColor)).withSize(15).withBounceMode(LarsonBounceValue.Center);
        io.setAnimation(larsonAnimation);
        currentPattern = Patterns.LARSON;
        break;

      case STROBE:
        io.setColor(currentColor);
        strobeAnimation = new StrobeAnimation(8, 399).withSlot(4).withColor(new RGBWColor(currentColor)).withFrameRate(ANIMATION_SPEED);
        io.setAnimation(strobeAnimation);
        currentPattern = Patterns.STROBE;
        break;

    }
  }

  public Command setLedColor(Color color) {
    return runOnce(() -> {
      currentColor = color;
      io.setColor(color);
    }

    );

  }


  //if we want to switch colors based on robot mode, do it here


  public Command intakeFlash() {
    return runOnce(() -> {
      currentColor = Color.kWhite;
      setPattern(Patterns.STROBE);
      }).andThen(wait(500));
  }


  public enum Patterns {
    SOLID, FADE, LARSON, STROBE;
  }


}
