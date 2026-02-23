package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.PracticeBotConstants;

public class FlywheelIOPB implements FlywheelIO {
  private static final double GEAR_RATIO = 1.0; // FIXME: set actual gear ratio
  private static final double KP = 3.0;
  private static final double KI = 0.0;
  private static final double KD = 0.1;
  private static final double KA = 0.0;
  private static final double KG = 0.0;
  private static final double KS = 3.0;
  private static final double KV = 0.008;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 200.0;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 100.0;

  private final TalonFX[] motors = {
    new TalonFX(PracticeBotConstants.FLYWHEEL_RIGHT_ID, PracticeBotConstants.CANBUS),
    new TalonFX(PracticeBotConstants.FLYWHEEL_LEFT_ID, PracticeBotConstants.CANBUS)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  public FlywheelIOPB() {
    Slot0Configs slot0Configs = rightConfig.Slot0;
    slot0Configs.kA = KA;
    slot0Configs.kD = KD;
    slot0Configs.kG = KG;
    slot0Configs.kI = KI;
    slot0Configs.kP = KP;
    slot0Configs.kS = KS;
    slot0Configs.kV = KV;

    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    for (TalonFX i : motors) {
      i.getConfigurator().apply(defaultConfig);
    }

    rightConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    rightConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    leftConfig = rightConfig.clone();
    // do not edit right configs after cloning
    leftConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    motors[1].getConfigurator().apply(leftConfig);
    motors[1].setNeutralMode(NeutralModeValue.Coast);
    motors[0].getConfigurator().apply(rightConfig);
    motors[0].setNeutralMode(NeutralModeValue.Coast);

    boolean oddFollower = true;
    for (int i = 1; i < motors.length; i++) {
      motors[i].setControl(
          new Follower(
              PracticeBotConstants.FLYWHEEL_RIGHT_ID,
              (oddFollower ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned)));
      oddFollower = !oddFollower;
    }
  }

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
  VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

  @Override
  public void setVelocity(double rpm) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(rpm).in(RotationsPerSecond)));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      inputs.statorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();
      inputs.supplyCurrents[i] = motors[i].getSupplyCurrent().getValueAsDouble();
      inputs.positions[i] = motors[i].getPosition().getValueAsDouble();
      inputs.velocities[i] =
          RotationsPerSecond.of(motors[i].getVelocity().getValueAsDouble()).in(RPM);
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }
}
