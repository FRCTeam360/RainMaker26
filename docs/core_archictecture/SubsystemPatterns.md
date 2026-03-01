# Subsystem Design Patterns

This document describes the common patterns that emerge across all subsystem classes in our codebase. These patterns apply to subsystems that extend `SubsystemBase` and interact with the SuperStructure coordination layer.

**Note:** For information about the IO layer pattern and AdvantageKit integration, see [AdvantageKit_Subsystem_Guide.md](./AdvantageKit_Subsystem_Guide.md). For the complete architecture overview, see [SubsystemArchitecture.md](./SubsystemArchitecture.md).

---

## Table of Contents

1. [Core Pattern](#core-pattern)
2. [File Structure](#file-structure)
3. [State Management Pattern](#state-management-pattern)
4. [Control Mode Pattern](#control-mode-pattern)
5. [Periodic Lifecycle](#periodic-lifecycle)
6. [Command Factory Pattern](#command-factory-pattern)
7. [Common Code Sections](#common-code-sections)
8. [Naming Conventions](#naming-conventions)
9. [Examples from Codebase](#examples-from-codebase)

---

## Core Pattern

All subsystems in our codebase follow this template:

```java
public class <Subsystem> extends SubsystemBase {
  // 1. Constants (tuning values with units)
  // 2. IO layer (hardware abstraction)
  // 3. State enums (wanted + internal states)
  // 4. State variables
  // 5. Constructor (dependency injection)
  // 6. State machine API (getters/setters)
  // 7. State machine logic (private update/apply methods)
  // 8. IO delegation methods (public hardware control)
  // 9. Command factories (returns Command objects)
  // 10. periodic() override
}
```

This creates a **consistent, predictable structure** across all subsystems.

---

## File Structure

Every subsystem lives in its own package with the IO layer:

```
subsystems/
  <Subsystem>/
    <Subsystem>.java          ← Main subsystem class (extends SubsystemBase)
    <Subsystem>IO.java        ← IO interface (@AutoLog inputs)
    <Subsystem>IO<Config>.java  ← Hardware implementations (PB, WB, Sim)
```

**Example:**

```
subsystems/
  HopperRoller/
    HopperRoller.java         ← Extends SubsystemBase
    HopperRollerIO.java       ← Interface
    HopperRollerIOPB.java     ← Practice Bot hardware
    HopperRollerIOSim.java    ← Simulation
```

---

## State Management Pattern

### Two-Layer State System

Subsystems use **two levels of states**:

#### 1. Wanted States (External Interface)

These represent **high-level behaviors** requested by the SuperStructure or commands:

```java
public enum <Subsystem>WantedStates {
  OFF,          // Safe/idle state
  <ACTION_1>,   // First behavior
  <ACTION_2>,   // Second behavior
  // ...
}
```

**Naming convention:**
- Use `UPPER_SNAKE_CASE`
- Use present participles for actions: `INTAKING`, `SHOOTING`, `SPINNING_UP`
- Use adjectives/nouns for conditions: `IDLE`, `STOWED`, `READY`

#### 2. Internal States (Execution Phase)

These represent the **current execution status** within a wanted behavior:

```java
public enum <Subsystem>InternalStates {
  OFF,                    // Inactive
  MOVING_TO_SETPOINT,     // In progress
  AT_SETPOINT,            // Achieved target
  // ... subsystem-specific states
}
```

**When to use internal states:**
- Complex behaviors with multiple phases (e.g., Flywheel: `SPINNING_UP` → `AT_SETPOINT` → `RECOVERING`)
- Need to track progress toward a goal
- Coordination with other subsystems requires knowing readiness

**When NOT to use internal states:**
- Simple on/off behaviors (e.g., HopperRoller just mirrors wanted state)
- No multi-phase execution needed

### State Variables

Every subsystem has these state-tracking fields:

```java
// State variables
private <Subsystem>WantedStates wantedState = <Subsystem>WantedStates.OFF;
private <Subsystem>InternalStates currentState = <Subsystem>InternalStates.OFF;
private <Subsystem>InternalStates previousState = <Subsystem>InternalStates.OFF;
private ControlState controlState = ControlState.SUPERSTRUCTURE;
```

**Pattern rules:**
- Initialize to safe defaults (`OFF`, `IDLE`, etc.)
- `previousState` enables edge detection (state transitions)
- `controlState` defaults to `SUPERSTRUCTURE` mode

---

## Control Mode Pattern

Subsystems support **dual control modes** via the `ControlState` enum:

```java
public enum ControlState {
  SUPERSTRUCTURE,  // Controlled by SuperStructure state machine
  INDEPENDENT      // Controlled directly by commands
}
```

### Implementation

```java
public void setControlState(ControlState controlState) {
  this.controlState = controlState;
}

@Override
public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs("<Subsystem>", inputs);

  // Only run state machine when controlled by SuperStructure
  if (controlState == ControlState.SUPERSTRUCTURE) {
    updateState();
    applyState();
  }

  // Always log state
  Logger.recordOutput("Subsystems/<Subsystem>/WantedState", wantedState);
  Logger.recordOutput("Subsystems/<Subsystem>/CurrentState", currentState);
  Logger.recordOutput("Subsystems/<Subsystem>/ControlState", controlState);
}
```

### Usage Pattern

**SuperStructure Mode (default):**

```java
// SuperStructure sets wanted states, subsystem state machine runs automatically
intakePivot.setWantedState(IntakePivotWantedStates.DEPLOYED);
// State machine handles: OFF → MOVING_TO_SETPOINT → AT_SETPOINT
```

**Independent Mode (testing/tuning):**

```java
// Switch to independent mode
superStructure.setControlState(ControlState.INDEPENDENT);

// Commands directly control hardware (bypasses state machine)
intakePivot.setPositionCommand(() -> 45.0).schedule();
```

**When to use each mode:**
- `SUPERSTRUCTURE`: Competition, autonomous, coordinated behaviors
- `INDEPENDENT`: Tuning PID, testing individual subsystems, debugging hardware

---

## Periodic Lifecycle

Every subsystem follows this exact `periodic()` pattern:

```java
@Override
public void periodic() {
  // Step 1: Read sensor inputs from hardware (via IO layer)
  io.updateInputs(inputs);
  Logger.processInputs("<Subsystem>", inputs);

  // Step 2: Run state machine (only if controlled by SuperStructure)
  if (controlState == ControlState.SUPERSTRUCTURE) {
    updateState();   // Compute internal state from wanted state + sensors
    applyState();    // Send commands to hardware based on internal state
  }

  // Step 3: Log all state for debugging
  Logger.recordOutput("Subsystems/<Subsystem>/WantedState", wantedState);
  Logger.recordOutput("Subsystems/<Subsystem>/CurrentState", currentState);
  Logger.recordOutput("Subsystems/<Subsystem>/PreviousState", previousState);
  Logger.recordOutput("Subsystems/<Subsystem>/ControlState", controlState);
}
```

**Critical ordering:**
1. **Read** inputs first (establishes current state)
2. **Compute** state transitions (logic based on current inputs)
3. **Apply** outputs (hardware commands)
4. **Log** everything (for replay/debugging)

**Why this order matters:**
- Reading inputs first ensures decisions are based on fresh sensor data
- Updating state before applying ensures outputs match current logic
- Logging at the end captures the final state after all processing

---

## Command Factory Pattern

Subsystems provide **command factory methods** that return `Command` objects for independent control:

### Pattern: runEnd

For continuous actions that need cleanup:

```java
public Command setDutyCycleCommand(double dutyCycle) {
  return runEnd(
    () -> setDutyCycle(dutyCycle),  // Execute: runs every 20ms
    () -> setDutyCycle(0.0)          // End: cleanup when interrupted
  );
}
```

**Usage:**
```java
// Runs roller while button held, stops when released
controller.a().whileTrue(hopperRoller.setDutyCycleCommand(0.75));
```

### Pattern: runOnce

For one-time configuration changes:

```java
public Command setPositionCommand(double position) {
  return runOnce(() -> setPosition(position));
}
```

**Usage:**
```java
// Sets position once when button pressed
controller.b().onTrue(intakePivot.setPositionCommand(90.0));
```

### Pattern: Supplier-Based Commands

For dynamic values (from joysticks, calculations, etc.):

```java
public Command setDutyCycleCommand(DoubleSupplier dutyCycleSupplier) {
  return runEnd(
    () -> setDutyCycle(dutyCycleSupplier.getAsDouble()),
    () -> setDutyCycle(0.0)
  );
}
```

**Usage:**
```java
// Duty cycle tracks joystick Y axis
controller.leftStick().whileTrue(
  intake.setDutyCycleCommand(() -> controller.getLeftY())
);
```


**Naming convention:**
- Action verbs: `setDutyCycleCommand`, `setPositionCommand`, `spinUpCommand`
- Always include `Command` suffix to distinguish from direct methods

---

## Common Code Sections

### Section 1: Constants

Tuning values with **units in names**:

```java
// Constants
private static final double ROLLER_DUTY_CYCLE = 0.75;
private static final double PREVENT_JAM_DUTY_CYCLE = -0.1;
private static final double UNJAMMING_DUTY_CYCLE = -0.8;
private static final double TOLERANCE_DEGREES = 2.0;
private static final double STALL_CURRENT_AMPS = 25.0;
private static final double SPINUP_TIMEOUT_SECONDS = 3.0;
```

**Rules:**
- Use `private static final` (compile-time constants)
- Include units: `_DEGREES`, `_RPM`, `_AMPS`, `_SECONDS`, `_METERS`, `_MPS`
- Group by purpose (speeds, tolerances, timeouts)

### Section 2: IO Layer

Hardware abstraction via dependency injection:

```java
// IO fields
private final <Subsystem>IO io;
private final <Subsystem>IOInputsAutoLogged inputs = new <Subsystem>IOInputsAutoLogged();
```

**Rules:**
- `io` is `final` (set in constructor, never changes)
- `inputs` uses `AutoLogged` suffix (generated by AdvantageKit)
- Never access hardware directly — always through `io`

### Section 3: State Machine API

Public interface for state control:

```java
// State machine methods
public <Subsystem>InternalStates getState() {
  return currentState;
}

public void setWantedState(<Subsystem>WantedStates state) {
  wantedState = state;
}

public void setControlState(ControlState controlState) {
  this.controlState = controlState;
}
```

**Pattern:**
- Getters return internal state (for coordination checks)
- Setters accept wanted state (external requests)
- Control state setter (switches between modes)

### Section 4: State Machine Logic

Private methods implementing state transitions:

```java
private void updateState() {
  previousState = currentState;
  
  switch (wantedState) {
    case <STATE_1>:
      // Logic to determine internal state
      currentState = <InternalState>;
      break;
    // ...
  }
}

private void applyState() {
  switch (currentState) {
    case <INTERNAL_STATE_1>:
      // Send commands to hardware via IO
      io.setDutyCycle(0.5);
      break;
    // ...
  }
}
```

**Pattern:**
- `updateState()` is **pure logic** (no hardware calls)
- `applyState()` is **pure I/O** (minimal logic)
- Always save `previousState` first (for edge detection)

### Section 5: IO Delegation

Public methods that wrap IO calls:

```java
// IO delegation methods
public void setDutyCycle(double dutyCycle) {
  io.setDutyCycle(dutyCycle);
}

public void setPosition(double position) {
  io.setPosition(position);
}

public void stop() {
  io.setDutyCycle(0.0);
}
```

**Purpose:**
- Provides clean API for commands
- Can add validation/logging if needed
- Abstracts IO layer from external callers

---

## Naming Conventions

### Subsystem Classes

```java
public class Flywheel extends SubsystemBase { }
public class IntakePivot extends SubsystemBase { }
public class HopperRoller extends SubsystemBase { }
```

**Pattern:** PascalCase, descriptive noun, **no "Subsystem" suffix**

### State Enums

```java
public enum FlywheelWantedStates { }
public enum FlywheelInternalStates { }
```

**Pattern:**
- `<Subsystem>WantedStates` for external interface
- `<Subsystem>InternalStates` for execution phase (if needed)

### State Values

```java
// Actions (present participles)
INTAKING, SHOOTING, SPINNING_UP, EJECTING, RECOVERING

// Conditions (adjectives/nouns)
IDLE, OFF, STOWED, AT_SETPOINT, READY

// Multi-phase (descriptive compounds)
MOVING_TO_SETPOINT, SWITCHING_AGITATE_TARGET_HIGH
```

**Pattern:** `UPPER_SNAKE_CASE`, descriptive, spell out words fully

### Methods

```java
// State machine API
public void setWantedState(FlywheelWantedStates state)
public FlywheelInternalStates getState()
public void setControlState(ControlState controlState)

// Private state logic
private void updateState()
private void applyState()

// IO delegation
public void setDutyCycle(double duty)
public void setPosition(double position)
public void stop()

// Command factories
public Command setDutyCycleCommand(double duty)
public Command spinUpCommand(double rpm)
```

**Pattern:**
- State API: `set<Property>`, `get<Property>`
- State logic: `updateState`, `applyState` (private)
- IO delegation: verb-first, camelCase
- Commands: action + `Command` suffix

---

## Examples from Codebase

### Simple Subsystem (HopperRoller)

**Characteristics:**
- No complex internal states (wanted = current)
- Simple on/off behaviors
- Minimal state logic

```java
public class HopperRoller extends SubsystemBase {
  private static final double ROLLER_DUTY_CYCLE = 0.75;
  private static final double PREVENT_JAM_DUTY_CYCLE = -0.1;

  private final HopperRollerIO io;
  private final HopperRollerIOInputsAutoLogged inputs = new HopperRollerIOInputsAutoLogged();

  public enum HopperRollerStates {
    OFF, ROLLING, UNJAMMING, PREVENT_JAM
  }

  private HopperRollerStates wantedState = HopperRollerStates.OFF;
  private HopperRollerStates currentState = HopperRollerStates.OFF;
  private HopperRollerStates previousState = HopperRollerStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  public HopperRoller(HopperRollerIO io) {
    this.io = io;
  }

  private void updateState() {
    previousState = currentState;
    // Simple: wanted state directly becomes current state
    currentState = wantedState;
  }

  private void applyState() {
    switch (currentState) {
      case ROLLING:
        setDutyCycle(ROLLER_DUTY_CYCLE);
        break;
      case PREVENT_JAM:
        setDutyCycle(PREVENT_JAM_DUTY_CYCLE);
        break;
      // ...
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("HopperRoller", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }

    Logger.recordOutput("Subsystems/HopperRoller/WantedState", wantedState);
    Logger.recordOutput("Subsystems/HopperRoller/CurrentState", currentState);
    Logger.recordOutput("Subsystems/HopperRoller/ControlState", controlState);
  }
}
```

**When to use this pattern:**
- Simple mechanisms with discrete modes
- No sensor-based state transitions
- No multi-phase behaviors

### Complex Subsystem (Flywheel)

**Characteristics:**
- Multi-phase internal states
- Sensor-based transitions (velocity, debouncing)
- External setpoint suppliers

```java
public class Flywheel extends SubsystemBase {
  private static final double TOLERANCE_RPM = 150.0;
  private static final double BALL_FIRED_DEBOUNCE_SECONDS = 0.04;

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private DoubleSupplier shootVelocitySupplier = () -> 0.0;

  public enum FlywheelWantedStates {
    IDLE, SHOOTING, COASTING
  }

  public enum FlywheelInternalStates {
    OFF, SPINNING_UP, AT_SETPOINT, RECOVERING, UNDER_SHOOTING, COAST
  }

  private final Debouncer ballFiredDebouncer =
      new Debouncer(BALL_FIRED_DEBOUNCE_SECONDS, DebounceType.kFalling);

  private FlywheelWantedStates wantedState = FlywheelWantedStates.IDLE;
  private FlywheelInternalStates currentState = FlywheelInternalStates.OFF;
  private FlywheelInternalStates previousState = FlywheelInternalStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  private long launchCount = 0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void setShootVelocitySupplier(DoubleSupplier shootVelocitySupplier) {
    this.shootVelocitySupplier = shootVelocitySupplier;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING: {
        double targetRPM = shootVelocitySupplier.getAsDouble();
        boolean atSetpoint = atSetpoint(targetRPM);
        boolean ballFired = (previousState == AT_SETPOINT) && !atSetpoint;

        if (ballFired) {
          launchCount++;
          currentState = RECOVERING;
        } else if (atSetpoint) {
          currentState = AT_SETPOINT;
        } else if (previousState == RECOVERING) {
          currentState = RECOVERING;
        } else {
          currentState = SPINNING_UP;
        }
        break;
      }
      // ... other cases
    }
  }

  private boolean atSetpoint(double targetRPM) {
    boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
    return ballFiredDebouncer.calculate(inTolerance);
  }

  private void applyState() {
    switch (currentState) {
      case SPINNING_UP:
      case RECOVERING:
        setSpinupVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      case AT_SETPOINT:
        setHoldVelocityControl(shootVelocitySupplier.getAsDouble());
        break;
      // ...
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }

    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Flywheel/LaunchCount", launchCount);
  }
}
```

**When to use this pattern:**
- Multi-phase execution (spinup → hold → recover)
- Sensor-based state transitions (velocity, position, current)
- Need to track execution progress for coordination
- Debouncing required to filter noise

---

## Summary

### Key Principles

1. **Consistent Structure**: All subsystems follow the same 10-section template
2. **State Separation**: Wanted (external) vs. Internal (execution phase)
3. **Dual Control**: SuperStructure mode vs. Independent mode
4. **IO Abstraction**: Hardware accessed only through injected IO interface
5. **Logging First**: Every state, input, and output logged for debugging

### Decision Tree

**Simple subsystem (no internal states):**
- Discrete on/off behaviors → Use HopperRoller pattern
- Wanted state directly becomes current state
- Minimal `updateState()` logic

**Complex subsystem (multi-phase):**
- Multi-step execution → Use Flywheel pattern
- Sensor-based transitions → Add debouncers
- Coordination readiness checks → Track internal states

### Common Mistakes to Avoid

❌ **Don't access hardware directly**
```java
// Bad
private final TalonFX motor = new TalonFX(10);
motor.setControl(...);

// Good
io.setDutyCycle(0.5);
```

❌ **Don't skip state logging**
```java
// Bad - no visibility into state machine
@Override
public void periodic() {
  updateState();
  applyState();
}

// Good - full logging
@Override
public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs("Subsystem", inputs);
  if (controlState == ControlState.SUPERSTRUCTURE) {
    updateState();
    applyState();
  }
  Logger.recordOutput("Subsystems/Subsystem/WantedState", wantedState);
  Logger.recordOutput("Subsystems/Subsystem/CurrentState", currentState);
}
```

❌ **Don't use unnamed constants**
```java
// Bad
if (inputs.velocity > 500) { ... }

// Good
private static final double MIN_VELOCITY_RPM = 500.0;
if (inputs.velocity > MIN_VELOCITY_RPM) { ... }
```

❌ **Don't forget previousState tracking**
```java
// Bad - can't detect edges
private void updateState() {
  currentState = ...;
}

// Good - edge detection possible
private void updateState() {
  previousState = currentState;
  boolean ballFired = (previousState == AT_SETPOINT) && !atSetpoint();
  // ...
}
```

---

## Checklist for New Subsystems

- [ ] Extends `SubsystemBase`
- [ ] Has IO interface with `@AutoLog` inputs
- [ ] Has at least one IO implementation (Real or Sim)
- [ ] Has wanted state enum
- [ ] Has internal state enum (if multi-phase behavior)
- [ ] Constructor takes `IO` interface parameter
- [ ] Has `setWantedState()`, `getState()`, `setControlState()` methods
- [ ] Has private `updateState()` and `applyState()` methods
- [ ] `periodic()` calls `io.updateInputs()` first
- [ ] `periodic()` checks `controlState` before running state machine
- [ ] `periodic()` logs all states
- [ ] Constants have units in names
- [ ] Command factory methods for independent control
- [ ] Follows file/naming conventions
