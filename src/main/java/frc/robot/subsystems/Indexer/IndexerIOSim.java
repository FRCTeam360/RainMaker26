// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationConstants;

public class IndexerIOSim implements IndexerIO {
  /** Creates a new IndexerIOSim. */

  private double gearRatio = 10.0;
  private DCMotor gearbox = DCMotor.getNeo550(1);
   private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  private final double kG = 0.75;  // Gravity compensation

  private DigitalInput sensor = new DigitalInput(0);
  private DIOSim sensorSim = new DIOSim(sensor);

  private final LoggedNetworkNumber tunableSetpoint = new LoggedNetworkNumber("/Tuning/Indexer/SetpointRotations", 0.0);
  private final LoggedNetworkBoolean tuningEnabled = new LoggedNetworkBoolean("/Tuning/Indexer/Enabled", false);

  private final TalonFX motorControllerSim = new TalonFX(SimulationConstants.INTAKE_PIVOT_MOTOR);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final LinearSystem<N1, N1, N1> plant =
    LinearSystemId.createFlywheelSystem(gearbox, 0.00113951385, 1.0); // TODO : MOI is last year's but i don't think that one was correct either

  private final FlywheelSim indexerSim =
    new FlywheelSim(
      plant,
      gearbox,
      0.01
    );

  public IndexerIOSim() {}

  public void updateInputs(IndexerIOInputs inputs) {
    simEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    indexerSim.setInput(simMotor.getSpeed() * RobotController.getBatteryVoltage());
    indexerSim.update(0.02);
    simEncoder.setDistance(simMotor.getPosition());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(indexerSim.getCurrentDrawAmps()));
    
    inputs.voltage = indexerMotor.getVoltage();
    inputs.position = simMotor.getPosition();
    inputs.statorCurrent = indexerSim.getCurrentDrawAmps();
    inputs.velocity = indexerSim.getAngularVelocityRPM();
    inputs.sensor = sensor.get();
  }

  public void setDutyCycle(double dutyCycle) {
    simMotor.setSpeed(dutyCycle);
  }
}
