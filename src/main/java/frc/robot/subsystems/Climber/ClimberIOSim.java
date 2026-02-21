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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {

  private static final double MEASUREMENT_STD_DEV_METERS = 0.01;
  private DCMotor gearbox = DCMotor.getNEO(1);
  private Encoder climberEncoder = new Encoder(4, 5);

  private final PWMSparkMax climberMotor = new PWMSparkMax(5);

  private final double JKgMetersSquared = 0.00113951385;

  private final double ENCODER_TICKS_PER_REVOLUTION = 4096;

  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, JKgMetersSquared, 1.0);


    //FIXME: Assign real values. Ensure it uses the constructor mentioning carriage and drum
  private final ElevatorSim climberSim =
    new ElevatorSim(gearbox, JKgMetersSquared, JKgMetersSquared, JKgMetersSquared, JKgMetersSquared, JKgMetersSquared, false, JKgMetersSquared, null);
  

  private final EncoderSim simClimberEncoder = new EncoderSim(climberEncoder);
  private final PWMSim simClimberMotor = new PWMSim(climberMotor);

  /** Creates a new ClimberIOSim. */
  public ClimberIOSim() {}

  public void updateInputs(ClimberIOInputs inputs) {
    simClimberEncoder.setDistancePerPulse(
        2.0
            * Math.PI
            * (Units.inchesToMeters(2.0))
            / ENCODER_TICKS_PER_REVOLUTION); // divided by 4096 to convert the encoder's raw rotational data into meters.
    climberSim.setInput(simClimberMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simClimberEncoder.setDistance(simClimberMotor.getPosition());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    inputs.climberLeftPosition = simClimberMotor.getPosition();
    inputs.climberRightPosition = simClimberMotor.getPosition();

    inputs.climberLeftVelocity = climberSim.getAngularVelocityRPM();    
    inputs.climberRightVelocity = climberSim.getAngularVelocityRPM();

    inputs.climberLeftDutyCycle = climberSim.getInputVoltage();
    inputs.climberRightDutyCycle = climberSim.getInputVoltage();

  }

  public void setLeftDutyCycle(double dutyCycle) {
    simClimberMotor.setSpeed(dutyCycle);
  }

  public void setRightDutyCycle(double dutyCycle) {
    simClimberMotor.setSpeed(dutyCycle);
  }

  public void setLeftPosition(double position) {
    simClimberMotor.setPosition(position);
  }

  public void setRightPosition(double position) {
    simClimberMotor.setPosition(position);
  }
}
