// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOPB implements HoodIO {
  private static final double GEAR_RATIO = 3.0 / 1.0 * 170.0 / 10.0;
  // 1/3 * 170/10
  private static final double KP = 300.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KA = 0.0;
  private static final double KG = 0.15; // .15
  private static final double KS = 0.2;
  private static final double KV = 0.0;
  private static final double FORWARD_SOFT_LIMIT_DEGREES = 47.0;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 25.0;
  private static final double MOTION_MAGIC_ACCELERATION_RPS2 = 4.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY_RPS = 2.0;
  private static final double MOTION_MAGIC_JERK_RPS3 = 1200.0;
  private final TalonFXS hoodMotor =
      new TalonFXS(Constants.PracticeBotConstants.HOOD_ID, Constants.PracticeBotConstants.CANBUS);
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

  private final MotionMagicVoltage motionMagicPosition = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;

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

    config.ExternalFeedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(FORWARD_SOFT_LIMIT_DEGREES);
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION_RPS2)
        .withMotionMagicCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY_RPS)
        .withMotionMagicJerk(MOTION_MAGIC_JERK_RPS3);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotor.getConfigurator().apply(config);

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    statorCurrentSignal = hoodMotor.getStatorCurrent();
    supplyCurrentSignal = hoodMotor.getSupplyCurrent();
    motorVoltageSignal = hoodMotor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionSignal,
        velocitySignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        motorVoltageSignal);
    hoodMotor.optimizeBusUtilization();

    setZero();
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
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        motorVoltageSignal);
    inputs.position = Units.rotationsToDegrees(positionSignal.getValueAsDouble());
    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = Units.rotationsToDegrees(velocitySignal.getValueAsDouble());
    inputs.voltage = motorVoltageSignal.getValueAsDouble();
  }

  public void setDutyCycle(double dutyCycle) {
    hoodMotor.set(dutyCycle);
  }
}
