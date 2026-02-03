package frc.robot.subsystems.Turret;

// note: I believe this is right. For "Implement Turret Subsystem"
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretIO;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    /** Creates a new Turret. */
    public Turret(TurretIO io) {
        // PID
        this.io = io;
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public void setEncoder(double position) {
        io.setEncoder(position);
    }

    public void stop() {
        io.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }
}
