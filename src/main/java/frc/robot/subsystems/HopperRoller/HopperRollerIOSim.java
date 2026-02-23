// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HopperRoller;

import com.revrobotics.sim.SparkFlexSim;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HopperRollerIOSim implements HopperRollerIO {
  private static final double GEAR_RATIO = 1.0;
  private static final DCMotor GEARBOX = DCMotor.getNEO(1);
  private static final double MOI_KG_M2 = 0.0513951385;

  // AdvantageScope tuning (sim-only, under /Tuning table)
  private final LoggedNetworkNumber targetDutyCycle =
      new LoggedNetworkNumber("/Tuning/HopperRoller/targetDutyCycle", 0.0);
  private final LoggedNetworkBoolean tuningEnabled =
      new LoggedNetworkBoolean("/Tuning/HopperRoller/Enabled", false);

  // Motor and control
  private final SparkFlex motorControllerSim =
      new SparkFlex(SimulationConstants.HOPPER_ROLLER_MOTOR, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();

  // SparkFlex simulation object
  private final SparkFlexSim sparkSim = new SparkFlexSim(motorControllerSim, GEARBOX);

  // Flywheel simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(GEARBOX, MOI_KG_M2, GEAR_RATIO);
  private final FlywheelSim hopperRollerSim = new FlywheelSim(plant, GEARBOX, GEAR_RATIO);

  public HopperRollerIOSim() {
    configureMotor();
    motorControllerSim.set(0.0);
  }

  private void configureMotor() {
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);
    motorConfig.smartCurrentLimit(40);
  }

  @Override
  public void updateInputs(HopperRollerIOInputs inputs) {
    if (tuningEnabled.get()) {
      double targetDuty = targetDutyCycle.get();
      motorControllerSim.set(Math.max(-1, Math.min(1, targetDuty)));
    }

    double commandedDutyCycle = motorControllerSim.get();
    double busVoltage = RoboRioSim.getVInVoltage();
    double appliedVoltage = commandedDutyCycle * busVoltage;

    hopperRollerSim.setInputVoltage(appliedVoltage);
    hopperRollerSim.update(0.02);

    sparkSim.iterate(hopperRollerSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperRollerSim.getCurrentDrawAmps()));

    inputs.position = 0.0;
    inputs.velocity = hopperRollerSim.getAngularVelocityRPM();
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = hopperRollerSim.getCurrentDrawAmps();
    inputs.supplyCurrent = hopperRollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motorControllerSim.set(dutyCycle);
  }
}
