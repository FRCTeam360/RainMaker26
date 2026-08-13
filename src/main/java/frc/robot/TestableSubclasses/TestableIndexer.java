package frc.robot.TestableSubclasses;

import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;

public class TestableIndexer extends  Indexer {
    TestableIndexer(IndexerIO io) {
        super(io);
    }
    public void runUpdateStateForTesting() {
        updateState();
    }
    public void runApplyStateForTesting() {
        applyState();
    }
}
