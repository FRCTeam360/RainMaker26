

package frc.robot.subsystems.IntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SoftLimitConfig;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot.IntakePivotIO.IntakePivotIOInputs;

public class IntakePivotIOPB implements IntakePivotIO {
  private final TalonFX intakePivot =
      new TalonFX(
          Constants.WoodBotConstants.INTAKE_PIVOT_ID, Constants.WoodBotConstants.CANBUS_NAME);
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private final SoftLimitConfig softLimitConfig = new SoftLimitConfig();

  /** Creates a new IntakePivotIOWB. */
  public IntakePivotIOPB() {
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    intakePivot.getConfigurator().apply(config);
    intakePivot.setNeutralMode(NeutralModeValue.Brake);

    currentLimitConfig.StatorCurrentLimit = 120.0;
    currentLimitConfig.SupplyCurrentLimit = 60.0;

    softLimitConfig.forwardSoftLimitEnabled(true);
    softLimitConfig.forwardSoftLimit(0.0);
    softLimitConfig.reverseSoftLimitEnabled(true);
    softLimitConfig.reverseSoftLimit(50.0);

    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
    final double kA = 0.0;
    final double kG = 0.0;
    final double kFF = 0.0;

    intakePivot.getConfigurator().apply(currentLimitConfig);
  }

  public void setPosition(double value) {
    intakePivot.setPosition(value);
  }

  public void setDutyCycle(double value) {
    intakePivot.set(value);
  }

  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = intakePivot.getPosition().getValueAsDouble();
    inputs.statorCurrent = intakePivot.getStatorCurrent().getValueAsDouble();
    inputs.velocity = intakePivot.getVelocity().getValueAsDouble();
    inputs.voltage = intakePivot.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = intakePivot.getSupplyCurrent().getValueAsDouble();
  }
}
