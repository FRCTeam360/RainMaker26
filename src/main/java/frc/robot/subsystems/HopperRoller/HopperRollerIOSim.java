// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import com.revrobotics.sim.SparkFlexSim;
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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SimulationConstants;

public class HopperRollerIOSim implements HopperRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final double MOI = 0.0008; // kg·m²

  private final DCMotor gearbox = DCMotor.getNeoVortex(1);

  private final SparkFlex motorControllerSim =
      new SparkFlex(SimulationConstants.HOPPER_ROLLER_MOTOR, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();
  private final SparkFlexSim sparkSim = new SparkFlexSim(motorControllerSim, gearbox);
  private final SparkClosedLoopController closedLoopController;

  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, MOI, GEAR_RATIO);
  private final FlywheelSim rollerSim = new FlywheelSim(plant, gearbox, GEAR_RATIO);

  public HopperRollerIOSim() {
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);
    motorConfig.smartCurrentLimit(70, 50);

    motorConfig.closedLoop.p(0.0002).i(0.0).d(0.0);
    motorConfig.closedLoop.feedForward.kV(0.0019).kS(0.04);

    closedLoopController = motorControllerSim.getClosedLoopController();
    motorControllerSim.set(0.0);
  }

  @Override
  public void updateInputs(HopperRollerIOInputs inputs) {
    double commandedDutyCycle = motorControllerSim.get();
    double busVoltage = RoboRioSim.getVInVoltage();
    double appliedVoltage = commandedDutyCycle * busVoltage;

    rollerSim.setInputVoltage(appliedVoltage);
    rollerSim.update(SimulationConstants.SIM_TICK_RATE_S);

    sparkSim.iterate(
        rollerSim.getAngularVelocityRPM(),
        RoboRioSim.getVInVoltage(),
        SimulationConstants.SIM_TICK_RATE_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));

    inputs.position = 0.0;
    inputs.velocity = rollerSim.getAngularVelocityRPM();
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = rollerSim.getCurrentDrawAmps();
    inputs.supplyCurrent = rollerSim.getCurrentDrawAmps();
    inputs.motorConnected = true;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motorControllerSim.set(dutyCycle);
  }

  @Override
  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
