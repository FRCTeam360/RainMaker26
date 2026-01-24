package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import frc.robot.Constants.SimulationConstants;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class IndexerIOSim implements IndexerIO {
  // Motor constants
  private double gearRatio = 1.0;
  private DCMotor gearbox = DCMotor.getNeo550(1);
  private final double kS = 0.1;
  private final double kV = 0.12;
  private final double kA = 0.0;
  private final double kG = 0.0; // No gravity compensation for flywheel
  private final double moi = 0.00113951385; // Moment of inertia in kg·m²

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber tunableKp = new LoggedNetworkNumber("/Tuning/Indexer/kP", 0.1);
  private final LoggedNetworkNumber tunableKi = new LoggedNetworkNumber("/Tuning/Indexer/kI", 0.0);
  private final LoggedNetworkNumber tunableKd = new LoggedNetworkNumber("/Tuning/Indexer/kD", 0.0);
  private final LoggedNetworkNumber tunableSetpoint = new LoggedNetworkNumber("/Tuning/Indexer/SetpointRotations", 0.0);
  private final LoggedNetworkBoolean tuningEnabled = new LoggedNetworkBoolean("/Tuning/Indexer/Enabled", false);

  // Motor and control (using SparkMax like the real hardware)
  private final SparkMax motorControllerSim = new SparkMax(SimulationConstants.INDEXER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder encoder = motorControllerSim.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // Sensor simulation
  private final DigitalInput sensor = new DigitalInput(0);
  private final DIOSim sensorSim = new DIOSim(sensor);

  // Flywheel simulation
  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(gearbox, moi, gearRatio);
  private final FlywheelSim indexerSim = new FlywheelSim(plant, gearbox, gearRatio);

  public IndexerIOSim() {
    // Configure SparkMax with PID and current limits
    configureMotor();

    // Initialize motor sim state to match flywheel sim initial state
    encoder.setPosition(indexerSim.getAngularPositionRotations());
    // Note: SparkMax sim doesn't have direct velocity setting
  }

  private void configureMotor() {
    // Configure motor to match real hardware
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);

    // Configure PID gains (use tunable defaults)
    motorConfig.pid(kP = tunableKp.get(), kI = tunableKi.get(), kD = tunableKd.get());

    // Apply configuration
    motorControllerSim.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double kP, kI, kD; // Store current PID values for tuning

  public void updateInputs(IndexerIOInputs inputs) {
    // --- AdvantageScope tuning (sim-only) ---
    if (tuningEnabled.get()) {
      // Apply tunable PID gains
      motorConfig.pid(kP = tunableKp.get(), kI = tunableKi.get(), kD = tunableKd.get());
      motorControllerSim.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Command the tunable setpoint using position control
      encoder.setPosition(tunableSetpoint.get());
    }
    // Step 1: Get the commanded voltage from motor and apply to simulation
    double appliedVoltage = motorControllerSim.get() * motorControllerSim.getBusVoltage();
    indexerSim.setInputVoltage(appliedVoltage);

    // Step 2: Update the simulation by one timestep
    indexerSim.update(0.02);

    // Step 3: Update the motor sim state with the new simulated values
    encoder.setPosition(indexerSim.getAngularPositionRotations());
    // Note: SparkMax sim doesn't have direct velocity setting, so we'll update it in the next cycle
    // Step 4: Update battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            indexerSim.getCurrentDrawAmps()));
            
    // Step 5: Update sensor simulation (simple: triggered when piece is at index
    // position)

    // For now, simulate sensor based on position (you can customize this logic)
    sensorSim.setValue(indexerSim.getAngularPositionRotations() % 1.0 > 0.5);
    // Step 6: Read all inputs from the SIMULATED VALUES (source of truth)
    inputs.position = indexerSim.getAngularPositionRotations();
    inputs.velocity = indexerSim.getAngularVelocityRPM();
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = indexerSim.getCurrentDrawAmps();
    inputs.supplyCurrent = indexerSim.getCurrentDrawAmps();
    inputs.sensor = sensorSim.getValue();
  }

  /**
   * Set position using PID control.
   * 
   * @param positionRotations Target position in rotations
   */
  public void setPosition(double positionRotations) {
    // For SparkMax, we use the built-in PID controller reference
    motorControllerSim.getCANSparkMax().getPIDController().setReference(positionRotations, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }

  /**
   * Set velocity using closed-loop velocity control.
   * 
   * @param velocityRPS Target velocity in rotations per second
   */
  public void setVelocity(double velocityRPS) {
    // For SparkMax, we use the built-in PID controller reference
    motorControllerSim.getCANSparkMax().getPIDController().setReference(velocityRPS * 60.0, com.revrobotics.CANSparkMax.ControlType.kVelocity); // Convert to RPM
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }
}