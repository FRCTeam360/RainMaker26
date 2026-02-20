package frc.robot.subsystems.Shooter.Flywheel;

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

  private final TalonFX[] motors = {
    new TalonFX(PracticeBotConstants.FLYWHEEL_RIGHT_ID, PracticeBotConstants.CANBUS),
    new TalonFX(PracticeBotConstants.FLYWHEEL_LEFT_ID, PracticeBotConstants.CANBUS)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  public FlywheelIOPB() {
    double kP = 3.0;
    double kI = 0.0;
    double kD = 0.1;
    double kA = 0.0;
    double kG = 0.0;
    double kS = 3.0;
    double kV = 0.008;

    Slot0Configs slot0Configs = rightConfig.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    for (TalonFX i : motors) {
      i.getConfigurator().apply(defaultConfig);
    }

    rightConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    rightConfig.CurrentLimits.StatorCurrentLimit = 200.0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
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
    double rps = rpm / 60.0;
    motors[0].setControl(velocityTorqueCurrent.withVelocity(rps));
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
      // velocities are now in RPM
      inputs.velocities[i] = motors[i].getVelocity().getValueAsDouble() * 60.0;
      inputs.voltages[i] = motors[i].getMotorVoltage().getValueAsDouble();
    }
  }
}
