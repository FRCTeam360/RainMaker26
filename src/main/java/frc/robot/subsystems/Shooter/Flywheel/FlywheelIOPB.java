package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.PracticeBotConstants;

public class FlywheelIOPB implements FlywheelIO {
  private static final double GEAR_RATIO = 1.0;
  private static final double KP = 20.0;
  private static final double KI = 0.0;
  private static final double KD = 0.1;
  private static final double KA = 0.0;
  private static final double KG = 0.0;
  private static final double KS = 3.0;
  private static final double KV = 0.0;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 200.0;
  private static final double MAX_NEGATIVE_TORQUE_CURRENT = -200.0;
  private static final double MAX_POSITIVE_TORQUE_CURRENT = STATOR_CURRENT_LIMIT_AMPS;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
  private static final double PEAK_REVERSE_VOLTAGE_VOLTS = -2.0;

  private final TalonFX[] motors = {
    new TalonFX(PracticeBotConstants.FLYWHEEL_RIGHT_ID, PracticeBotConstants.CANBUS),
    new TalonFX(PracticeBotConstants.FLYWHEEL_LEFT_ID, PracticeBotConstants.CANBUS)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  private final StatusSignal<Current> rightStatorCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;

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

    rightConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = MAX_POSITIVE_TORQUE_CURRENT;
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = MAX_NEGATIVE_TORQUE_CURRENT;

    rightConfig.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE_VOLTS;

    leftConfig = rightConfig.clone();
    // do not edit right configs after cloning
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

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

    rightStatorCurrentSignal = motors[0].getStatorCurrent();
    rightSupplyCurrentSignal = motors[0].getSupplyCurrent();
    rightPositionSignal = motors[0].getPosition();
    rightVelocitySignal = motors[0].getVelocity();
    rightMotorVoltageSignal = motors[0].getMotorVoltage();

    leftStatorCurrentSignal = motors[1].getStatorCurrent();
    leftSupplyCurrentSignal = motors[1].getSupplyCurrent();
    leftPositionSignal = motors[1].getPosition();
    leftVelocitySignal = motors[1].getVelocity();
    leftMotorVoltageSignal = motors[1].getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        rightVelocitySignal,
        rightMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
  VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

  @Override
  public void setSpinupVelocityControl(double rpm) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(rpm).in(RotationsPerSecond)));
  }

  @Override
  public void setHoldVelocityControl(double rpm) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(rpm).in(RotationsPerSecond)));
  }

  @Override
  public void setCoastVelocityControl(double rpm) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(rpm).in(RotationsPerSecond)));
  }

  @Override
  public void setDutyCycle(double duty) {
    motors[0].set(duty);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        rightVelocitySignal,
        rightMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal);

    inputs.statorCurrents[0] = rightStatorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrents[0] = rightSupplyCurrentSignal.getValueAsDouble();
    inputs.positions[0] = rightPositionSignal.getValueAsDouble();
    inputs.velocities[0] = RotationsPerSecond.of(rightVelocitySignal.getValueAsDouble()).in(RPM);
    inputs.voltages[0] = rightMotorVoltageSignal.getValueAsDouble();

    inputs.statorCurrents[1] = leftStatorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrents[1] = leftSupplyCurrentSignal.getValueAsDouble();
    inputs.positions[1] = leftPositionSignal.getValueAsDouble();
    inputs.velocities[1] = RotationsPerSecond.of(leftVelocitySignal.getValueAsDouble()).in(RPM);
    inputs.voltages[1] = leftMotorVoltageSignal.getValueAsDouble();
  }
}
