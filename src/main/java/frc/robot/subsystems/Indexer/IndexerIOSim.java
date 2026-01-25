package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOSim implements IndexerIO {
  // Motor constants
  private double gearRatio = 1.0;
  private DCMotor gearbox = DCMotor.getNeo550(1);
  private final double moi = 0.00113951385; // Moment of inertia in kg·m²

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber tunableSetpoint =
      new LoggedNetworkNumber("/Tuning/Indexer/SetpointRotations", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/Indexer/Enabled", false);

  // Motor and control (using SparkMax like the real hardware)
  private final SparkMax motorControllerSim =
      new SparkMax(SimulationConstants.INDEXER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder encoder = motorControllerSim.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // Sensor simulation
  private final DigitalInput sensor = new DigitalInput(0);
  private final DIOSim sensorSim = new DIOSim(sensor);

  // Flywheel simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, moi, gearRatio);
  private final FlywheelSim indexerSim = new FlywheelSim(plant, gearbox, gearRatio);

  public IndexerIOSim() {
    // Configure SparkMax with PID and current limits
    configureMotor();

    // Initialize everything to 0
    motorControllerSim.set(0.0);
    encoder.setPosition(0.0);
  }

  private void configureMotor() {
    // Configure motor to match real hardware
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);

    // Apply configuration
    motorControllerSim.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double kP, kI, kD; // Store current PID values for tuning

  public void updateInputs(IndexerIOInputs inputs) {
    // Get the commanded voltage from motor controller
    double motorOutput = motorControllerSim.get();

    double appliedVoltage = motorOutput * 12.0; // Use 12V as bus voltage

    // Update simulation
    indexerSim.setInputVoltage(appliedVoltage);
    indexerSim.update(0.02);

    // Get simulated values directly
    double velocity = indexerSim.getAngularVelocityRPM();
    double position = indexerSim.getAngularAccelerationRadPerSecSq();

    // Update encoder position to match simulation
    encoder.setPosition(position);

    // Simple sensor simulation
    sensorSim.setValue(position % 1.0 > 0.5);

    // Set inputs
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = indexerSim.getCurrentDrawAmps();
    inputs.supplyCurrent = indexerSim.getCurrentDrawAmps();
    inputs.sensor = sensorSim.getValue();
  }

  /**
   * Set position (not typically used for flywheel indexer).
   *
   * @param positionRotations Target position in rotations
   */
  public void setPosition(double positionRotations) {
    // For SparkMax, set the encoder position
    encoder.setPosition(positionRotations);
  }

  /**
   * Set velocity using closed-loop velocity control for flywheel.
   *
   * @param velocityRPS Target velocity in rotations per second
   */
  public void setVelocity(double velocityRPS) {
    // Convert RPS to duty cycle (simplified for simulation)
    double targetRPM = velocityRPS * 60;
    motorControllerSim.set(Math.max(-1, Math.min(1, targetRPM / 3000.0))); // Assuming max ~3000 RPM
  }

  @Override
  public void setDutyCycle(double value) {
    motorControllerSim.set(value);
  }
}
