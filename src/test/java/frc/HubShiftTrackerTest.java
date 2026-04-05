package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.HubShiftTracker;
import frc.robot.subsystems.HubShiftTrackerSimplified;
import frc.robot.subsystems.ShiftValues;
import frc.robot.subsystems.HubShiftTracker.MatchPhase;
import frc.robot.utils.RobotUtils.ActiveHub;

public class HubShiftTrackerTest {
    // Input double matchTime
    // Input Alliance autoWinner
    // Output teleop shift number
    // Output current phase
    // Output active hub
    // Output autoWinner
    // Output time until phase change

    // For Transition, test autoWinner and driverStationMatchPhase
    @Test
    void testStartOfTransitionLostAuto() {
        double matchTime = 140;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.TRANSITION, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_1_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfTransitionWonAuto() {
        double matchTime = 140;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.TRANSITION, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.TRANSITION_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testTransitionLostAutoEndOfShift() {
        double matchTime = 140;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.TRANSITION, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_1_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testTransitionWonAutoEndOfShift() {
        double matchTime = 140;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.TRANSITION, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.TRANSITION_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift1WonAuto() {
        double matchTime = 130;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(1, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.TRANSITION_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift1WonAuto() {
        double matchTime = 130;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(1, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_1_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift2LostAuto() {
        double matchTime = 105;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(2, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_2_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift2WonAuto() {
        double matchTime = 105;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(2, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_2_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift2LostAuto() {
        double matchTime = 105;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(2, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_2_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift2WonAuto() {
        double matchTime = 105;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(2, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_2_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift3LostAuto() {
        double matchTime = 80;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(3, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_3_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift3WonAuto() {
        double matchTime = 80;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(3, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_3_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift3LostAuto() {
        double matchTime = 80;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(3, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_3_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift3WonAuto() {
        double matchTime = 80;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOLOSER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(3, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.SHIFT_3_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift4WonAuto() {
        double matchTime = 55;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(4, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_START_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartOfShift4LostAuto() {
        double matchTime = 55;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(4, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_START_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testEndOfShift4WonAuto() {
        double matchTime = 55;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(4, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_START_SECONDS_SHOOTING, values);
    }

    @Test
    void testEndOfShift4LostAuto() {
        double matchTime = 55;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.AUTOWINNER, values.activeHub);
        assertEquals(MatchPhase.TELEOP, values.currentPhase);
        assertEquals(false, values.isOurHubActive);
        assertEquals(4, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_START_SECONDS_SHOOTING, values);
    }

    @Test
    void testStartOfEndgameLostAuto() {
        double matchTime = 30;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, false);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.ENDGAME, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }

    @Test
    void testStartWonAuto() {
        double matchTime = 30;
        ShiftValues values = HubShiftTrackerSimplified.getTeleopShiftValues(matchTime, true);
        assertEquals(ActiveHub.BOTH, values.activeHub);
        assertEquals(MatchPhase.ENDGAME, values.currentPhase);
        assertEquals(true, values.isOurHubActive);
        assertEquals(0, values.teleopShiftNumber);
        assertEquals(matchTime - HubShiftTrackerSimplified.ENDGAME_END_SECONDS_SHOOTING, values.timeUntilShootingPhaseChange);
    }




    // between transition and shift 1, test gracPeriod and timeOfFlight
    // For shift 1, test autoWinner, and matchTime and driverStationMatchPhase
    // between shift 1 and 2, test gracePeriod and timeOfFlight
    // For shift 2, test autoWinner, and matchTime and driverStationMatchPhase
    // between shift2 and shift3, test gracePeriod and timeofFlight
    // For shift 3, test autoWinner, and matchTime and driverStationMatchPhase
    // between shift3 and shift4, test gracePeriod and timeofFlight
    // For shift 4, test autoWinner, and matchTime and driverStationMatchPhase
    // between shift4 and endgame, test gracePeriod and timeOfFlight
    // For endgame, test matchTime and driverStationMatchPhase, gracePeriod and timeOfFlight
}
