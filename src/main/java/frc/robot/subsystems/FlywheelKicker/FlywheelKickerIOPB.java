

package frc.robot.subsystems.FlywheelKicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.PracticeBotConstants;

public class FlywheelKickerIOPB implements FlywheelKickerIO {
  /** Creates a new FlywheelKickerIOWB. */
  private final SparkMax flywheelkickerMotor =
      new SparkMax(Constants.PracticeBotConstants.FLYWHEEL_KICKER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = flywheelkickerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController closedLoopController;

  private final DigitalInput sensor =
      new DigitalInput(PracticeBotConstants.FLYWHEEL_KICKER_SENSOR_ID);

  public FlywheelKickerIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    sparkMaxConfig.smartCurrentLimit(40);

    sparkMaxConfig.closedLoop.p(0.0002).i(0.0).d(0.0);
    sparkMaxConfig.closedLoop.feedForward.kV(0.0021).kS(0.04);

    flywheelkickerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = flywheelkickerMotor.getClosedLoopController();
  }

  public void updateInputs(FlywheelKickerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = flywheelkickerMotor.getOutputCurrent();
    inputs.supplyCurrent =
        flywheelkickerMotor.getOutputCurrent()
            * flywheelkickerMotor.getAppliedOutput(); // TODO: check if
    // this is right
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = flywheelkickerMotor.getBusVoltage() * flywheelkickerMotor.getAppliedOutput();
    inputs.sensor = sensor.get();
  }

  public void setDutyCycle(double dutyCycle) {
    flywheelkickerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
