// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ClimberIOSim implements ClimberIO {

  private DCMotor gearbox = DCMotor.getNEO(1);
  private Encoder climberEncoder = new Encoder(4, 5);

  private final PWMSparkMax climberMotor = new PWMSparkMax(5);

  private final double JKgMetersSquared = 0.00113951385;
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(
          gearbox,
          JKgMetersSquared,
          1.0); // TODO: find actual MOI, old TODO, use if still applicable

  private final FlywheelSim climberSim =
      new FlywheelSim(
          plant, //
          gearbox, // gearbox
          0.01);

  private final EncoderSim simClimberEncoder = new EncoderSim(climberEncoder);
  private final PWMSim simClimberMotor = new PWMSim(climberMotor);

  /** Creates a new ClimberIOSim. */
  public ClimberIOSim() {}

  public void updateInputs(ClimberIOInputs inputs) {
    simClimberEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    climberSim.setInput(simClimberMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simClimberEncoder.setDistance(simClimberMotor.getPosition());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    inputs.climberPosition = simClimberMotor.getPosition();
    inputs.climberVelocity = climberSim.getAngularVelocityRPM();
    inputs.climberDutyCycle = climberSim.getInputVoltage();
  }

  public void setDutyCycle(double dutyCycle) {
    simClimberMotor.setSpeed(dutyCycle);
  }

  public void setPosition(double position) {
    simClimberMotor.setPosition(position);
  }
}
