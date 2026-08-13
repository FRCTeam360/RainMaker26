package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import frc.robot.subsystems.Indexer.IndexerIO;
import org.junit.jupiter.api.Test;

public class TestIndexer {

  private static class RecordingIndexerIO implements IndexerIO {
    private double lastDutyCycle = Double.NaN;
    private double lastVelocityRpm = Double.NaN;

    @Override
    public void setDutyCycle(double dutyCycle) {
      lastDutyCycle = dutyCycle;
    }

    @Override
    public void setVelocity(double velocity) {
      lastVelocityRpm = velocity;
    }
  }

  private static class TestableIndexer extends Indexer {
    TestableIndexer(IndexerIO io) {
      super(io);
    }

    public void runUpdateStateForTest() {
      super.updateState();
    }

    public void runApplyStateForTest() {
      super.applyState();
    }
  }

  @Test
  void testIndexingState() {
    RecordingIndexerIO io = new RecordingIndexerIO();
    TestableIndexer indexer = new TestableIndexer(io);

    indexer.setWantedState(IndexerStates.INDEXING);
    indexer.runUpdateStateForTest();
    indexer.runApplyStateForTest();

    assertEquals(IndexerStates.INDEXING, indexer.getState());
    assertEquals(3000.0, io.lastVelocityRpm);
  }

  @Test
  void testAssistIntakingState() {
    RecordingIndexerIO io = new RecordingIndexerIO();
    TestableIndexer indexer = new TestableIndexer(io);

    indexer.setWantedState(IndexerStates.ASSIST_INTAKING);
    indexer.runUpdateStateForTest();
    indexer.runApplyStateForTest();

    assertEquals(IndexerStates.ASSIST_INTAKING, indexer.getState());
    assertEquals(-0.15, io.lastDutyCycle);
  }

  @Test
  void testReversingState() {
    RecordingIndexerIO io = new RecordingIndexerIO();
    TestableIndexer indexer = new TestableIndexer(io);

    indexer.setWantedState(IndexerStates.REVERSING);
    indexer.runUpdateStateForTest();
    indexer.runApplyStateForTest();

    assertEquals(IndexerStates.REVERSING, indexer.getState());
    assertEquals(-0.35, io.lastDutyCycle);
  }

  @Test
  void testOffStateStopsIndexer() {
    RecordingIndexerIO io = new RecordingIndexerIO();
    TestableIndexer indexer = new TestableIndexer(io);

    indexer.setWantedState(IndexerStates.OFF);
    indexer.runUpdateStateForTest();
    indexer.runApplyStateForTest();

    assertEquals(IndexerStates.OFF, indexer.getState());
    assertEquals(0.0, io.lastDutyCycle);
  }
}
