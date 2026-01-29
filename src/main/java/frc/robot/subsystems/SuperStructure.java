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
        IDLE, //everything is stopped when nothing else happens
        DEFENSE, //driver holds defense button -> less desired velocitu moving latterally, more into rotation in drivetrain
        X_OUT, //hold down button to x out wheels or press once and wheels stop X-ing out when moved
        AUTO_ALIGHN,//aligns to a target
        X_OUT_SHOOTING,//when robot is aligned, ends when toggled off or shooting stops
        PREPAREING,//flywheel spins up and align to target
        READY_2_FIRE,//if robot aligned and flywheel up to proper speed
        FIRING,//while theres still fuel to shoot and ready to fire
        COLLECTING_FUEL,//while intake button pressed
           EJECTING// eject button
           ,PASSING    //has current zone, makes check for !current zone then passes to zone
    }

    private SuperStates wantedSuperState = SuperStates.IDLE;
    private SuperStates currentSuperState = SuperStates.IDLE;
    private SuperStates previousSuperState = SuperStates.IDLE;

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

            case IDLE:
                currentSuperState = SuperStates.IDLE;
                break;
        }
    }
    private void applyStates() {
        switch (currentSuperState) {
            case COLLECTING_FUEL:
                intakeFuel();
                break;

            case IDLE:
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
