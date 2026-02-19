package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IndexerIOPB implements IndexerIO {
  /** Creates a new IndexerIOPB. */
  private final SparkMax indexerMotor =
      new SparkMax(Constants.PracticeBotConstants.INDEXER_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = indexerMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  public IndexerIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(true);
    sparkMaxConfig.smartCurrentLimit(40);

    indexerMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.statorCurrent = indexerMotor.getOutputCurrent();
    inputs.supplyCurrent = indexerMotor.getOutputCurrent() * indexerMotor.getAppliedOutput(); 
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
  }

  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }
}
