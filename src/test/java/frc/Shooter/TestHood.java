package frc.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodInternalStates;
import frc.robot.subsystems.Shooter.Hood.Hood.HoodWantedStates;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import org.junit.jupiter.api.Test;

public class TestHood {
  private static class RecordingHoodIO implements HoodIO {
    private double lastDutyCycle = Double.NaN;

    @Override
    public void setDutyCycle(double dutyCycle) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setDutyCycle'");
    }

    @Override
    public void setPositionSmooth(double position) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPositionSmooth'");
    }

    @Override
    public void setPositionAggressive(double position) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setPositionAggressive'");
    }

    @Override
    public void setZero() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setZero'");
    }
  }

  private static class TestableHood extends Hood {
    TestableHood(HoodIO io) {
      super(io);
    }

    public void updateStateTest() {
      super.updateState();
    }

    public void applyStateTest() {
      super.applyState();
    }
  }

  @Test
  void testDuckedZeroingState() {
    RecordingHoodIO io = new RecordingHoodIO();
    TestableHood testHood = new TestableHood(io);

    testHood.setWantedState(HoodWantedStates.DUCKED);
    testHood.updateStateTest();

    assertEquals(HoodInternalStates.ZEROING, testHood.getState());
  }

  @Test
  void testIdleState() {
    RecordingHoodIO io = new RecordingHoodIO();
    TestableHood testHood = new TestableHood(io);

    testHood.setWantedState(HoodWantedStates.IDLE);
    testHood.updateStateTest();

    assertEquals(HoodInternalStates.OFF, testHood.getState());
  }

  @Test
  void testIfAtSetpoint() {
    RecordingHoodIO io = new RecordingHoodIO();
    TestableHood testHood = new TestableHood(io);

    testHood.setWantedState(HoodWantedStates.AIMING);
    testHood.updateStateTest();

    assertEquals(HoodInternalStates.AT_SETPOINT, testHood.getState());
  }

  @Test
  void testIfMoving() {
    RecordingHoodIO io = new RecordingHoodIO();
    TestableHood testHood = new TestableHood(io);

    testHood.setHoodAngleSupplier(() -> 500000);
    testHood.setWantedState(HoodWantedStates.AIMING);
    testHood.updateStateTest();

    assertEquals(HoodInternalStates.MOVING, testHood.getState());
  }

  @Test
  void testDefaultOffState() {
    RecordingHoodIO io = new RecordingHoodIO();
    TestableHood testHood = new TestableHood(io);

    testHood.updateStateTest();

    assertEquals(HoodInternalStates.OFF, testHood.getState());
  }
}
