// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class HoodIOPB implements HoodIO {
  // /** Creates a new HoodIOWB. */
  private final TalonFX hoodMotor = new TalonFX(Constants.PracticeBotConstants.HOOD_ID);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);

  public void setZero() {
    hoodMotor.setPosition(0);
  }

  public HoodIOPB() {
    double kP = 0.21;
    double kI = 0.0;
    double kD = 0.0;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 0.0;
    double kV = 0.0;

    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 24.0;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    // not legit vals yet stole from flywheel :sob:
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 100.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.withMotionMagicAcceleration(0.0)
        .withMotionMagicCruiseVelocity(0.0)
        .withMotionMagicJerk(0.0);
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    hoodMotor.getConfigurator().apply(config);
  }

  public void setPosition(double position) {
    hoodMotor.setControl(motionMagicPosition.withPosition(position));
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = hoodMotor.getPosition().getValueAsDouble();
    inputs.statorCurrent = hoodMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = hoodMotor.getVelocity().getValueAsDouble();
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    ;
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
