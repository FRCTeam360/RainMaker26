package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerIOInputs {
  public double voltage = 0.0;
  public double supplyCurrent = 0.0;
  public double statorCurrent = 0.0;
  public double velocity = 0.0;
  public double position = 0.0;
  // capacity sensor
  public boolean sensor = false;
  public double sensorProximity = 0.0;
  public boolean sensorActivated = false;
  //motor connection
  public boolean motorConnected = true;
}
