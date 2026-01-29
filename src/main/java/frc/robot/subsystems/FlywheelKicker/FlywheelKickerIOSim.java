// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FlywheelKicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.WoodBotConstants;

public class FlywheelKickerIOSim implements FlywheelKickerIO {
  /** Creates a new FlywheelKickerIOWB. */

  public void updateInputs(FlywheelKickerIOInputs inputs) {
  }

  public void setDutyCycle(double dutyCycle) {
    System.out.println("blah but flywheel kicker");
  }
}
