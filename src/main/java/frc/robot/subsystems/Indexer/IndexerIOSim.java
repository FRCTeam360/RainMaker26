package frc.robot.subsystems.Indexer;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOSim implements IndexerIO {
  // Motor constants
  private double gearRatio = 5.0;
  private DCMotor gearbox = DCMotor.getNEO(1);
  private final double moi = 0.0513951385; // Moment of inertia in kg·m²

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber targetDutyCycle =
      new LoggedNetworkNumber("/Tuning/Indexer/targetDutyCycle", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/Indexer/Enabled", false);

  // Motor and control (using SparkMax like the real hardware)
  private final SparkMax motorControllerSim =
      new SparkMax(SimulationConstants.INDEXER_MOTOR, MotorType.kBrushless);
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // SparkMax simulation object
  private final SparkMaxSim sparkSim = new SparkMaxSim(motorControllerSim, gearbox);

  // Flywheel simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, moi, gearRatio);
  private final FlywheelSim indexerSim = new FlywheelSim(plant, gearbox, gearRatio);

  public IndexerIOSim() {
    // Configure SparkMax with PID and current limits
    configureMotor();

    // Initialize motor to 0
    motorControllerSim.set(0.0);
  }

  private void configureMotor() {
    // Configure motor to match real hardware
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // --- AdvantageScope tuning (sim-only) ---
    if (tuningEnabled.get()) {
      // Command the tunable setpoint in RPM
      double targetDuty = targetDutyCycle.get();
      motorControllerSim.set(Math.max(-1, Math.min(1, targetDuty)));
    }

    // Step 1: Get commanded duty cycle directly (not from getAppliedOutput which includes noise)
    double commandedDutyCycle = motorControllerSim.get();
    double busVoltage = RoboRioSim.getVInVoltage();
    double appliedVoltage = commandedDutyCycle * busVoltage;

    // Step 2: Set the input voltage to the physics simulation
    indexerSim.setInputVoltage(appliedVoltage);

    // Step 3: Update the physics simulation
    indexerSim.update(0.02);

    // Step 4: Use SparkMaxSim.iterate() to update the Spark MAX with simulated values
    sparkSim.iterate(
        indexerSim.getAngularVelocityRPM(), // Motor velocity in RPM
        RoboRioSim.getVInVoltage(), // Simulated battery voltage
        0.02); // Time interval (20ms)
    // Step 5: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerSim.getCurrentDrawAmps()));

    // Step 6: Set inputs from simulated values (source of truth)
    inputs.position = 0.0; // Position not tracked for this flywheel
    inputs.velocity = indexerSim.getAngularVelocityRPM(); // in Velocity
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = indexerSim.getCurrentDrawAmps();
    inputs.supplyCurrent = indexerSim.getCurrentDrawAmps();
    inputs.sensor = false; // Sensor not actively used, ignore for now (defaults to false in Sim)
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }
}
