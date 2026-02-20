// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

  private final TalonFXS hoodMotor =
      new TalonFXS(Constants.PracticeBotConstants.HOOD_ID, Constants.PracticeBotConstants.CANBUS);
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

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

    config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

    config.ExternalFeedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(24.0); // FIXME: verify limit in degrees
    config.CurrentLimits.StatorCurrentLimit = 25.0;
    // NEO 550 has lower current capacity than Falcon 500
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
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
    inputs.velocity = RotationsPerSecond.of(hoodMotor.getVelocity().getValueAsDouble()).in(RPM);
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
