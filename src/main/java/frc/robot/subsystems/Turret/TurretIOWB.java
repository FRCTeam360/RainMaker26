package frc.robot.subsystems.Turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

public class TurretIOWB implements TurretIO {
    private final SparkMax turretMotor = new SparkMax(Constants.WoodBotConstants.TURRET_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = turretMotor.getEncoder();
    // assumes sparkMax
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    SparkClosedLoopController controller = turretMotor.getClosedLoopController();

    public void setEncoder(double position) {
        encoder.setPosition(position);
    }

    public TurretIOWB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);

        turretMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sparkMaxConfig.softLimit // if softLimits are needed
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(0)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(0);

        turretMotor.configure(
                sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = turretMotor.getClosedLoopController();
    }

    public void setPosition(double position) {
        controller.setSetpoint(position, ControlType.kPosition);
    }

    public void updateInputs(TurretIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.statorCurrent = turretMotor.getOutputCurrent();
        inputs.supplyCurrent = turretMotor.getOutputCurrent() * turretMotor.getAppliedOutput();
        inputs.velocity = encoder.getVelocity();
        inputs.voltage = turretMotor.getBusVoltage() * turretMotor.getAppliedOutput();
    }

    public void setDutyCycle(double dutyCycle) {
        turretMotor.set(dutyCycle);
    }

}
