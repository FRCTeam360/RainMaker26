// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

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

public class IndexerIOSim implements IndexerIO {
  /** Creates a new IndexerIOSim. */

  private DCMotor gearbox = DCMotor.getNeo550(1);
  private Encoder encoder = new Encoder(6, 7);
  private DigitalInput sensor = new DigitalInput(0);
  
  private final PWMSparkMax indexerMotor = new PWMSparkMax(5);

  private PWMSim simMotor = new PWMSim(indexerMotor);
  private EncoderSim simEncoder = new EncoderSim(encoder);
  private DIOSim sensorSim = new DIOSim(sensor);

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
