package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;

public class SuperStructure extends SubsystemBase {
    private final Intake intake;

    public enum SuperStates {
        STOPPED
    }

    private SuperStates wantedSuperState = SuperStates.STOPPED;
    private SuperStates currentSuperState = SuperStates.STOPPED;
    private SuperStates previousSuperState;


    private SuperStates setCurrentState() {
        previousSuperState = currentSuperState;
        switch (currentSuperState) {
            default:
                currentSuperState = SuperStates.STOPPED;
                break;
        }
        return currentSuperState;
    }

    private void applyStates(){
        switch (currentSuperState){
            case STOPPED:
                stopped();
                break;
        }
    }

    private void stopped(){
        //setWantedState(stoped)
    }

    public SuperStructure(Intake intake) {
        this.intake = intake;
    }
    @Override
    public void periodic(){
        Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
        Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
        Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);
        currentSuperState = setCurrentState();
    }

}
