package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CompBotConstants;

public class FlywheelIOCB implements FlywheelIO {
  private static final double GEAR_RATIO = 1.0;
  private static final double KP = 20.0;
  private static final double KI = 0.0;
  private static final double KD = 0.1;
  private static final double KA = 0.0;
  private static final double KG = 0.0;
  private static final double KS = 3.0;
  private static final double KV = 0.0;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 200.0;
  private static final double MAX_NEGATIVE_TORQUE_CURRENT = -40.0;
  private static final double MAX_POSITIVE_TORQUE_CURRENT = STATOR_CURRENT_LIMIT_AMPS;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
  private static final double PEAK_REVERSE_VOLTAGE_VOLTS = -2.0;

  private static final double TELEMETRY_UPDATE_FREQUENCY_HZ = 50.0;
  private static final double CONTROL_UPDATE_FREQUENCY_HZ = 500.0;

  private final TalonFX[] motors = {
    new TalonFX(CompBotConstants.FLYWHEEL_RIGHT_ID, CompBotConstants.CANBUS),
    new TalonFX(CompBotConstants.FLYWHEEL_LEFT_ID, CompBotConstants.CANBUS)
  };
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

  private final StatusSignal<Current> rightStatorCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightTorqueCurrentSignal;
  private final StatusSignal<Double> rightDutyCycleSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;

  public FlywheelIOCB() {
    rightConfig.Slot0.kP = KP;
    rightConfig.Slot0.kI = KI;
    rightConfig.Slot0.kD = KD;
    rightConfig.Slot0.kA = KA;
    rightConfig.Slot0.kG = KG;
    rightConfig.Slot0.kS = KS;
    rightConfig.Slot0.kV = KV;

    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    for (TalonFX motor : motors) {
      motor.getConfigurator().apply(defaultConfig);
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
    // do not edit rightConfig after cloning
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    motors[1].getConfigurator().apply(leftConfig);
    motors[1].setNeutralMode(NeutralModeValue.Coast);
    motors[0].getConfigurator().apply(rightConfig);
    motors[0].setNeutralMode(NeutralModeValue.Coast);

    motors[1].setControl(
        new Follower(CompBotConstants.FLYWHEEL_RIGHT_ID, MotorAlignmentValue.Opposed));

    rightStatorCurrentSignal = motors[0].getStatorCurrent();
    rightSupplyCurrentSignal = motors[0].getSupplyCurrent();
    rightPositionSignal = motors[0].getPosition();
    rightVelocitySignal = motors[0].getVelocity();
    rightMotorVoltageSignal = motors[0].getMotorVoltage();
    rightTorqueCurrentSignal = motors[0].getTorqueCurrent();
    rightDutyCycleSignal = motors[0].getDutyCycle();

    leftStatorCurrentSignal = motors[1].getStatorCurrent();
    leftSupplyCurrentSignal = motors[1].getSupplyCurrent();
    leftPositionSignal = motors[1].getPosition();
    leftVelocitySignal = motors[1].getVelocity();
    leftMotorVoltageSignal = motors[1].getMotorVoltage();

    // DutyCycle, MotorVoltage, and TorqueCurrent must stay enabled at high frequency on the leader
    // for the follower to operate correctly (per CTRE docs)
    BaseStatusSignal.setUpdateFrequencyForAll(
        CONTROL_UPDATE_FREQUENCY_HZ,
        rightMotorVoltageSignal,
        rightTorqueCurrentSignal,
        rightDutyCycleSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        TELEMETRY_UPDATE_FREQUENCY_HZ,
        rightVelocitySignal,
        rightStatorCurrentSignal,
        rightSupplyCurrentSignal,
        rightPositionSignal,
        leftVelocitySignal,
        leftMotorVoltageSignal,
        leftStatorCurrentSignal,
        leftSupplyCurrentSignal,
        leftPositionSignal);

    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  private final VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

  @Override
  public void setSpinupVelocityControl(double velocityRPM) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(velocityRPM).in(RotationsPerSecond)));
  }

  @Override
  public void setHoldVelocityControl(double velocityRPM) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(velocityRPM).in(RotationsPerSecond)));
  }

  @Override
  public void setCoastVelocityControl(double velocityRPM) {
    motors[0].setControl(velocityTorqueCurrent.withVelocity(RPM.of(velocityRPM).in(RotationsPerSecond)));
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
