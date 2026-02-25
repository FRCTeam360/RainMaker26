# Model-Based State Machine Test Framework

## Context

The codebase has 9 state machines (SuperStructure, ShooterStateMachine, Flywheel, Hood, FlywheelKicker, Intake, IntakePivot, Indexer, HopperRoller) with only 3 existing unit tests -- none covering state machine logic. As AI-assisted code generation expands on the team, we need automated validation that state machines behave correctly. This framework defines each state machine's expected behavior declaratively, then walks every transition path to catch dead states, missing guards, and incorrect outputs -- all running in CI via `./gradlew test`.

## What It Catches Today

- 6 unreachable SuperStates (DEFENSE, X_OUT, AUTO_ALIGN, X_OUT_SHOOTING, FIRING, EJECTING) silently mapping to IDLE
- `Indexer.INTAKING` is a dead state (commented out in SuperStructure)
- All 8 boolean combinations of the FIRING gate (flywheel ready + hood ready + aligned)
- Any future enum value added without updating the spec fails immediately

## Approach

**Test through the public API.** Each subsystem exposes `setWantedState()`, `getState()`, and `periodic()`. We call `periodic()` to drive the state machine, which requires AdvantageKit Logger to be started (solved with `Logger.start()` in `@BeforeAll` -- zero receivers = silent no-op mode).

**Recording IOs instead of plain Noops.** To verify what the state machine tells hardware to do, we create IO implementations that record every method call with arguments. For Flywheel and Hood, the Recording IOs also inject simulated sensor values (`inputs.velocities[0]`, `inputs.position`) to control the `atSetpoint()` guards.

## New Files

### Test files (`src/test/java/frc/robot/statemachine/`)

| File | Purpose |
|------|---------|
| `RecordingCall.java` | Record type capturing an IO method name + args |
| `FlywheelIORecording.java` | Records calls + injects simulated velocity |
| `HoodIORecording.java` | Records calls + injects simulated position |
| `FlywheelKickerIORecording.java` | Records calls |
| `IntakeIORecording.java` | Records calls |
| `IndexerIORecording.java` | Records calls |
| `HopperRollerIORecording.java` | Records calls |
| `IntakePivotIORecording.java` | Records calls |
| `StateMachineTestHelper.java` | One-time Logger.start() + CommandScheduler cleanup |
| `ShooterStateMachineTest.java` | Exhaustive 3-guard boolean testing (8 combos) |
| `FlywheelStateMachineTest.java` | Guarded transitions (atSetpoint tolerance) |
| `HoodStateMachineTest.java` | Guarded transitions (atSetpoint tolerance) |
| `FlywheelKickerStateMachineTest.java` | Simple 1:1 transitions + outputs |
| `IntakeStateMachineTest.java` | Transitions + duplicate output verification |
| `IntakePivotStateMachineTest.java` | Simple 1:1 transitions + outputs |
| `IndexerStateMachineTest.java` | Transitions + dead state (INTAKING) |
| `HopperRollerStateMachineTest.java` | Simple 1:1 transitions + outputs |
| `SuperStructureStateMachineTest.java` | Integration: 4 reachable states, 6 dead states, child subsystem outputs |

### Noop IOs (`src/main/java/frc/robot/subsystems/`)

5 new Noop IOs following the existing `HopperRollerIONoop` pattern:

| File | Pattern |
|------|---------|
| `Intake/IntakeIONoop.java` | `setDutyCycle(){}`, `setVelocity(){}`, `stop(){}` |
| `Indexer/IndexerIONoop.java` | `setDutyCycle(){}`, `setVelocity(){}` |
| `Shooter/Flywheel/FlywheelIONoop.java` | `setDutyCycle(){}`, `setVelocity(){}` |
| `Shooter/Hood/HoodIONoop.java` | `setDutyCycle(){}`, `setPosition(){}`, `setZero(){}` |
| `Shooter/FlywheelKicker/FlywheelKickerIONoop.java` | `setDutyCycle(){}`, `setVelocity(){}` |

## Recording IO Pattern

Each Recording IO implements the subsystem's IO interface, records calls in a `List<RecordingCall>`, and exposes `getCalls()` / `clearCalls()`. Flywheel and Hood also override `updateInputs()` to inject sensor values:

```java
public class FlywheelIORecording implements FlywheelIO {
    private final List<RecordingCall> calls = new ArrayList<>();
    private double simulatedVelocity = 0.0;

    public void setSimulatedVelocity(double rpm) { this.simulatedVelocity = rpm; }
    public List<RecordingCall> getCalls() { return List.copyOf(calls); }
    public void clearCalls() { calls.clear(); }

    @Override public void updateInputs(FlywheelIOInputs inputs) {
        Arrays.fill(inputs.velocities, simulatedVelocity);
    }
    @Override public void setDutyCycle(double duty) {
        calls.add(new RecordingCall("setDutyCycle", duty));
    }
    @Override public void setVelocity(double rpm) {
        calls.add(new RecordingCall("setVelocity", rpm));
    }
}
```

## Test Helper

```java
public final class StateMachineTestHelper {
    private static boolean loggerStarted = false;

    public static synchronized void ensureLoggerStarted() {
        if (!loggerStarted) {
            Logger.start();  // zero receivers = silent replay mode
            loggerStarted = true;
        }
    }
}
```

Every test class calls `StateMachineTestHelper.ensureLoggerStarted()` in `@BeforeAll` and `CommandScheduler.getInstance().unregisterAllSubsystems()` in `@AfterEach` to prevent cross-test contamination.

## Key Test Specifications

### ShooterStateMachineTest (highest priority)

Tests all 8 guard combinations for the FIRING transition:

| flywheel ready | hood ready | aligned | expected state |
|:-:|:-:|:-:|---|
| false | false | false | PREPARING |
| true | false | false | PREPARING |
| false | true | false | PREPARING |
| false | false | true | PREPARING |
| true | true | false | PREPARING |
| true | false | true | PREPARING |
| false | true | true | PREPARING |
| true | true | true | FIRING |

Also tests: IDLE to IDLE, output verification (PREPARING sets kicker IDLE, FIRING sets kicker KICKING).

Setup: construct Flywheel/Hood/FlywheelKicker with Recording IOs, inject sensor values to control guards.

### SuperStructureStateMachineTest

- Verifies 4 reachable states produce correct child subsystem states
- Verifies all 6 unimplemented SuperStates map to IDLE behavior
- Verifies INTAKING sets Intake.INTAKING + IntakePivot.DEPLOYED + ShooterStateMachine.IDLE
- Verifies SHOOT_AT_HUB/OUTPOST sets ShooterStateMachine.SHOOTING
- Verifies the FIRING gate: indexer/hopper only active when ShooterStateMachine reaches FIRING

Setup: all 7 Recording IOs, trivial ShotCalculators with identity interpolation maps.

### Flywheel/Hood Tests

- IDLE maps to OFF (output: `setDutyCycle(0.0)` / `setPosition(0.0)`)
- SHOOTING/AIMING + not at setpoint maps to MOVING (output: `setVelocity`/`setPosition` with target)
- SHOOTING/AIMING + at setpoint maps to AT_SETPOINT (output: same as MOVING)
- Edge: velocity exactly at tolerance boundary (100 RPM / 0.5 degrees)

### Simple Subsystem Tests

Each verifies: every wanted state produces correct current state and correct IO call. IndexerStateMachineTest additionally documents INTAKING as a dead state when driven by SuperStructure.

## Implementation Order

1. `StateMachineTestHelper.java` + `RecordingCall.java` (foundation)
2. 5 Noop IOs in `src/main/java/` (follow `HopperRollerIONoop` pattern)
3. 7 Recording IOs in `src/test/java/`
4. `HopperRollerStateMachineTest.java` (simplest -- validate framework works)
5. `FlywheelKickerStateMachineTest.java`, `IntakePivotStateMachineTest.java`
6. `IntakeStateMachineTest.java`, `IndexerStateMachineTest.java`
7. `FlywheelStateMachineTest.java`, `HoodStateMachineTest.java` (guarded)
8. `ShooterStateMachineTest.java` (3-guard exhaustive)
9. `SuperStructureStateMachineTest.java` (full integration)

## Design Decision: No Generic Walker Framework

After reviewing the state machines, direct JUnit test classes per state machine are recommended rather than a generic `StateMachineSpec`/`StateMachineTestWalker` abstraction. The state machines are similar but not uniform enough for a single generic walker to handle cleanly (SuperStructure delegates to ShooterStateMachine which delegates to child subsystems; Flywheel/Hood have sensor-driven guards; simple subsystems are 1:1 mappings). A generic framework would add complexity without proportional value for 9 state machines. Each test class is ~60-100 lines and self-contained. If the team adds more state machines in the future, the pattern is easy to replicate.

## Verification

1. `./gradlew test` -- all new tests pass
2. `./gradlew spotlessApply` -- formatting applied
3. `./gradlew jacocoTestReport` -- coverage report shows state machine code covered
4. `./gradlew build` -- full build succeeds
