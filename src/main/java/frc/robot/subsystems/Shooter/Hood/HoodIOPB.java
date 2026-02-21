// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class HoodIOPB implements HoodIO {
  private static final double GEAR_RATIO = 1.0; // FIXME: set actual gear ratio
  private static final double KP = 0.21;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KA = 0.0;
  private static final double KG = 0.0;
  private static final double KS = 0.0;
  private static final double KV = 0.0;
  private static final double FORWARD_SOFT_LIMIT_DEGREES = 24.0; // FIXME: verify limit
  private static final double STATOR_CURRENT_LIMIT_AMPS = 25.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  private final TalonFXS hoodMotor =
      new TalonFXS(Constants.PracticeBotConstants.HOOD_ID, Constants.PracticeBotConstants.CANBUS);
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

  private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);

  public void setZero() {
    hoodMotor.setPosition(0);
  }

  public HoodIOPB() {
    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kA = KA;
    slot0Configs.kD = KD;
    slot0Configs.kG = KG;
    slot0Configs.kI = KI;
    slot0Configs.kP = KP;
    slot0Configs.kS = KS;
    slot0Configs.kV = KV;

    config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

    config.ExternalFeedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(FORWARD_SOFT_LIMIT_DEGREES);
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    // NEO 550 has lower current capacity than Falcon 500
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.withMotionMagicAcceleration(0.0)
        .withMotionMagicCruiseVelocity(0.0)
        .withMotionMagicJerk(0.0);
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotor.getConfigurator().apply(config);
  }

  /**
   * Sets the hood position.
   *
   * @param positionDegrees target position in degrees
   */
  public void setPosition(double positionDegrees) {
    hoodMotor.setControl(
        motionMagicPosition.withPosition(Units.degreesToRotations(positionDegrees)));
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
    inputs.statorCurrent = hoodMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = Units.rotationsToDegrees(hoodMotor.getVelocity().getValueAsDouble());
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
