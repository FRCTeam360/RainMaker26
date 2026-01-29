package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;

public class SuperStructure extends SubsystemBase {
    private final Intake intake;
    private final Indexer indexer;

    public enum SuperStates {
        STOPPED,
        COLLECTING_FUEL,
           EJECTING,
           AUTO_SHOOTING
    }

    private SuperStates wantedSuperState = SuperStates.STOPPED;
    private SuperStates currentSuperState = SuperStates.STOPPED;
    private SuperStates previousSuperState = SuperStates.STOPPED;

    public SuperStructure(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
    }
    private void updateState() {
        previousSuperState = currentSuperState;

        switch (wantedSuperState) {
            case COLLECTING_FUEL:
                currentSuperState = SuperStates.COLLECTING_FUEL;
                break;

            case STOPPED:
                currentSuperState = SuperStates.STOPPED;
                break;
        }
    }
    private void applyStates() {
        switch (currentSuperState) {
            case COLLECTING_FUEL:
                intakeFuel();
                break;

            case STOPPED:
                stopped();
                break;
        }
    }

    private void intakeFuel() {
        intake.setWantedState(Intake.IntakeStates.COLLECTING_FUEL);
        indexer.setWantedState(Indexer.IndexerStates.COLLECTING_FUEL);
    }

    private void stopped() {
        intake.setWantedState(Intake.IntakeStates.OFF);
        indexer.setWantedState(Indexer.IndexerStates.OFF);
    }
    public Command setStateCommand(SuperStates superState) {
        return new InstantCommand(
            () -> setWantedSuperState(superState),
            this
        );
    }

    public void setWantedSuperState(SuperStates superState) {
        this.wantedSuperState = superState;
    }
    @Override
    public void periodic() {
        updateState();
        applyStates();

        Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState.toString());
        Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
        Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState.toString());
    }
}
