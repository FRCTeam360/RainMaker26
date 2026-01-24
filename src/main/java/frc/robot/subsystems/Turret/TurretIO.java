package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    /** Creates a new TurretIO. */
    @AutoLog
    public static class TurretIOInputs {
        // taken from HoodIO.java
        public double voltage = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;
        public double velocity = 0.0;
        public double position = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {
    }

    public void setDutyCycle(double dutyCycle);

    public void setPosition(double position);

    public void setEncoder(double position);
}
