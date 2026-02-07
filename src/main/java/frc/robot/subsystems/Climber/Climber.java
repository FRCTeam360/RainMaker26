// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(2.0, 3.0);

  private final ProfiledPIDController controller = 
    new ProfiledPIDController(1.0, 0.0, 0.0, m_constraints);

  public Climber() {
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motor.setInverted(false);
  }

  public void setClimbPosition(double goalClimbPosition) {
    controller.setGoal(goalClimbPosition);
  }

  @Override
  public void periodic() {
    double measurement = getClimbPosition();
    double output = controller.calculate(measurement);
  }

  public void setClimberOutput(double speed) {
        m_motor.set(speed);
    }

  public double getClimbPosition() {
    return 0;
  }

  public void stop() {
        m_motor.set(0);
    }
}
