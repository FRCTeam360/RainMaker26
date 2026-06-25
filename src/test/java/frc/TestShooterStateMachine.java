package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterStates;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterWantedStates;
import org.junit.jupiter.api.Test;

public class TestShooterStateMachine {
  @Test
  void TestSetWantedStateStandby() {
    ShooterStateMachine testShooterStateMachine =
        new ShooterStateMachine(null, null, null, null, null);
    testShooterStateMachine.setIsInAllianceZoneSupplier(() -> true);
    testShooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_SHOOTER);
    testShooterStateMachine.update();
    assertEquals(ShooterStates.STANDBY, testShooterStateMachine.getState());
  }
  @Test
  void TestSetWantedStateWaiting() {
    ShooterStateMachine testShooterStateMachine =
        new ShooterStateMachine(null, null, null, null, null);
    testShooterStateMachine.setIsInAllianceZoneSupplier(() -> false);
    testShooterStateMachine.setWantedState(ShooterWantedStates.PASSIVE_SHOOTER);
    testShooterStateMachine.update();
    assertEquals(ShooterStates.WAITING, testShooterStateMachine.getState());
  }
  //put stuff for more states ig
}
