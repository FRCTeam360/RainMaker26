package frc;

import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.IndexerStates;
import org.junit.Test;

public class TestIndexer {
  @Test
  void TestIndexingState() {
    Indexer indexer = new Indexer(null);
    indexer.setWantedState(IndexerStates.INDEXING);
    // find way to access updateState() and applyState() because they're protected
  }
}
