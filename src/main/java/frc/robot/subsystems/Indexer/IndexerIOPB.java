package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class IndexerIOPB implements IndexerIO {
  private static final double GEAR_RATIO = 1.0; // FIXME: set actual gear ratio
  private static final int CURRENT_LIMIT_AMPS = 40;
  private static final double KP = 0.0002;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double FF_KV = 0.0021;
  private static final double FF_KS = 0.04;

  /** Creates a new IndexerIOPB. */
  private final SparkFlex indexerMotor =
      new SparkFlex(Constants.PracticeBotConstants.INDEXER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
  private final SparkClosedLoopController closedLoopController;

  public IndexerIOPB() {
    sparkFlexConfig.idleMode(IdleMode.kBrake);
    sparkFlexConfig.inverted(true);
    sparkFlexConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

    sparkFlexConfig.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
    sparkFlexConfig.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

    sparkFlexConfig.closedLoop.p(KP).i(KI).d(KD);
    sparkFlexConfig.closedLoop.feedForward.kV(FF_KV).kS(FF_KS);

    indexerMotor.configure(
        sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = indexerMotor.getClosedLoopController();
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = indexerMotor.getOutputCurrent();
    inputs.supplyCurrent = 0;
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }

  public void setVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
  }
}
