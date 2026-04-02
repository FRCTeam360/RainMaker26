// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRoller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeRollerIOSim implements IntakeRollerIO {
  // Match PracticeBot (PB) config
  private static final double GEAR_RATIO = 1.0;
  private static final int CURRENT_LIMIT_AMPS = 55;
  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KV = 0.0019;
  private static final double KS = 0.04;
  private static final double MOI = 0.0008; // Moment of inertia in kg·m²
  private static final double SIMULATED_STALL_CURRENT_AMPS = 80.0;

  // Motor and control
  private final SparkFlex motorControllerSim =
      new SparkFlex(SimulationConstants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopController;

  // Sensor simulation
  private final DigitalInput sensor =
      new DigitalInput(SimulationConstants.INTAKE_ROLLER_SENSOR_PORT);
  private final DIOSim sensorSim = new DIOSim(sensor);

  // Jam disturbance injection for testing anti-jam logic
  private final LoggedNetworkBoolean injectJam =
      new LoggedNetworkBoolean("/Tuning/IntakeRoller/InjectJam", false);
  private final LoggedNetworkNumber jamDurationSeconds =
      new LoggedNetworkNumber("/Tuning/IntakeRoller/JamDurationSeconds", 1.0);

  private double jamStartTime = 0.0;
  private boolean jamActive = false;

  // Simulation
  private final DCMotor gearbox = DCMotor.getNeoVortex(1);
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, MOI, GEAR_RATIO);
  private final FlywheelSim intakeSim = new FlywheelSim(plant, gearbox, GEAR_RATIO);

  public IntakeRollerIOSim() {
    // Configure SparkFlex with PID matching PB
    configureMotor();

    // Initialize closed loop controller
    closedLoopController = motorControllerSim.getClosedLoopController();
  }

  private void configureMotor() {
    SparkFlexConfig config = new SparkFlexConfig();

    // Configure motor settings (match PB)
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);
    config.smartCurrentLimit(CURRENT_LIMIT_AMPS);

    // Configure encoder conversion factors
    config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    // Configure PID gains (match PB)
    config.closedLoop.p(KP).i(KI).d(KD);
    config.closedLoop.feedForward.kV(KV).kS(KS);

    // Apply configuration (no persist in sim)
    motorControllerSim.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    // Step 1: Get the commanded voltage from motor and apply to simulation
    double motorVoltage = motorControllerSim.get() * 12.0; // Assume 12V bus

    // Check if jam should be injected
    if (injectJam.get() && !jamActive) {
      jamStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      jamActive = true;
    }

    // Check if jam duration has elapsed
    if (jamActive
        && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - jamStartTime
            > jamDurationSeconds.get()) {
      jamActive = false;
      injectJam.set(false);
    }

    // Apply jam: zero out voltage to stall the motor
    double appliedVoltage = jamActive ? 0.0 : motorVoltage;

    // Step 2: Update the simulation by one timestep
    intakeSim.setInputVoltage(appliedVoltage);
    intakeSim.update(SimulationConstants.SIM_TICK_RATE_S);

    // Step 3: Simple sensor simulation - triggers based on velocity
    // Sensor activates when spinning fast enough (>100 RPM)
    sensorSim.setValue(Math.abs(intakeSim.getAngularVelocityRPM()) > 100);

    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));

    // Step 5: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.velocity[0] = intakeSim.getAngularVelocityRPM(); // SparkFlex encoder default is RPM
    inputs.voltage[0] = appliedVoltage;
    inputs.statorCurrent[0] =
        jamActive ? SIMULATED_STALL_CURRENT_AMPS : intakeSim.getCurrentDrawAmps();
    inputs.supplyCurrent = intakeSim.getCurrentDrawAmps();
    inputs.sensor = sensorSim.getValue();
    inputs.position[0] = 0.0; // Not tracked for flywheel
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.p(kP).i(kI).d(kD);
    pidConfig.closedLoop.feedForward.kV(kV).kS(kS);
    motorControllerSim.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoopController.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    motorControllerSim.set(0.0);
  }
}
