package frc.robot.subsystems;

import frc.robot.utils.RobotUtils.ActiveHub;

public class ShiftValues {
  public int teleopShiftNumber;
  public MatchPhase currentPhase;
  public ActiveHub activeHub;
  public boolean isOurHubActive;
  public double timeUntilShootingPhaseChange;

  public ShiftValues(
      int teleopShiftNumber,
      MatchPhase currentPhase,
      ActiveHub activeHub,
      boolean isOurHubActive,
      double timeUntilShootingPhaseChange) {
    this.teleopShiftNumber = teleopShiftNumber;
    this.currentPhase = currentPhase;
    this.activeHub = activeHub;
    this.isOurHubActive = isOurHubActive;
    this.timeUntilShootingPhaseChange = timeUntilShootingPhaseChange;
  }
}
