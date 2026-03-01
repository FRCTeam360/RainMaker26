# Subsystem Architecture Guide

This document explains how FRC Team 360's subsystems are structured and how they interact with each other.

---

## Table of Contents

1. [Overview](#overview)
2. [The IO Layer Pattern](#the-io-layer-pattern)
3. [State Machine Pattern](#state-machine-pattern)
4. [Dual Control Modes](#dual-control-modes)
5. [SuperStructure Coordination](#superstructure-coordination)
6. [Complete Code Structure](#complete-code-structure)
7. [Data Flow](#data-flow)
8. [File Organization](#file-organization)

---

## Overview

Our robot code follows a **layered architecture** that separates hardware abstraction, state management, and high-level coordination:

```
┌─────────────────────────────────────────┐
│       SuperStructure (Coordinator)       │  ← High-level game logic
├─────────────────────────────────────────┤
│  Individual Subsystems (State Machines)  │  ← Behavior logic
├─────────────────────────────────────────┤
│         IO Layer (Hardware I/O)          │  ← Hardware abstraction
├─────────────────────────────────────────┤
│    Hardware (Motors, Sensors, Encoders)  │  ← Physical components
└─────────────────────────────────────────┘
```

This architecture is adapted from **FRC Team 6328 (Mechanical Advantage)** and uses **AdvantageKit** for replay-based logging and testing.

---

## The IO Layer Pattern

### Purpose

The IO layer **abstracts hardware interaction** so subsystems can run on different robot configurations (real hardware, simulation, test fixtures) without changing subsystem logic.

### Components

Every hardware subsystem has three parts:

#### 1. **IO Interface** (`<Subsystem>IO.java`)

Defines the contract for hardware interaction:

```java
public interface IndexerIO {
  @AutoLog  // AdvantageKit generates logging inputs class
  public static class IndexerIOInputs {
    public double voltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean sensor = false;
    public double sensorProximity = 0.0;
    public boolean sensorActivated = false;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setVelocity(double velocity);
}
```

**Key features:**
- `@AutoLog` annotation generates `IndexerIOInputsAutoLogged` class for logging
- Inputs class contains **all sensor data** (readonly — only IO implementations write to it)
- Methods define **commands to hardware** (write operations)
- Default `updateInputs()` makes Noop implementations trivial

#### 2. **IO Implementations** (`<Subsystem>IO<Config>.java`)

Multiple implementations for different robot configurations:

| Implementation       | Purpose                                    | Example File             |
| -------------------- | ------------------------------------------ | ------------------------ |
| `IndexerIOPB`        | Practice Bot hardware (SparkMax NEO 550)   | `IndexerIOPB.java`       |
| `IndexerIOWB`        | WoodBot hardware (different motor config)  | `IndexerIOWB.java`       |
| `IndexerIOSim`       | Simulation (physics-based motor sim)       | `IndexerIOSim.java`      |
| _(no implementation needed)_ | Noop (subsystem not present on this robot) | Use default interface methods |

**Example: Real Hardware Implementation**

```java
public class IndexerIOPB implements IndexerIO {
  private static final double GEAR_RATIO = 9.0 / 1.0;
  private static final int CURRENT_LIMIT_AMPS = 40;

  private final SparkMax indexerMotor =
      new SparkMax(Constants.PracticeBotConstants.TWINDEXER_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = indexerMotor.getEncoder();

  public IndexerIOPB() {
    // Configure motor controllers (PID, current limits, inversions)
    sparkMaxConfig.idleMode(IdleMode.kCoast);
    sparkMaxConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);
    indexerMotor.configure(sparkMaxConfig, ...);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.voltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.statorCurrent = indexerMotor.getOutputCurrent();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    indexerMotor.set(dutyCycle);
  }
}
```

**Example: Simulation Implementation**

```java
public class IndexerIOSim implements IndexerIO {
  private final SparkFlex motorControllerSim = new SparkFlex(...);
  private final FlywheelSim indexerSim = new FlywheelSim(plant, gearbox, gearRatio);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = indexerSim.getAngularPositionRotations();
    inputs.velocity = indexerSim.getAngularVelocityRPM();
    inputs.voltage = sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    // Simulate battery draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerSim.getCurrentDrawAmps()));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motorControllerSim.set(dutyCycle);
    indexerSim.setInput(dutyCycle * RoboRioSim.getVInVoltage());
    indexerSim.update(0.020); // 20ms timestep
  }
}
```

#### 3. **Noop Implementation** (No Code Required)

When a subsystem doesn't exist on a robot config (e.g., Climber on WoodBot), use the **default interface methods**:

```java
// RobotContainer.java
Climber climber;
switch (Constants.getRobotType()) {
  case PRACTICE_BOT:
    climber = new Climber(new ClimberIOPB());
    break;
  case WOODBOT:
  case SIM:
  default:
    climber = new Climber(new ClimberIO() {}); // Noop — uses default methods
}
```

No special `ClimberIONoop` class needed — the interface defaults handle it.

---

## State Machine Pattern

### Two-Layer State System

Each subsystem uses **two levels of states**:

#### 1. **Wanted State** (External Commands)

Set by the **SuperStructure** or **commands** to request behavior:

```java
public enum IntakePivotWantedStates {
  OFF,
  STOWED,
  DEPLOYED,
  AGITATE_HOPPER,
  STACK_FUEL
}
```

#### 2. **Internal State** (Execution Phase)

Represents the subsystem's **current execution status** within the wanted behavior:

```java
public enum IntakePivotInternalStates {
  OFF,
  MOVING_TO_SETPOINT,
  AT_SETPOINT,
  SWITCHING_AGITATE_TARGET_HIGH,
  SWITCHING_AGITATE_TARGET_LOW
}
```

### State Machine Lifecycle

Every subsystem follows this pattern in `periodic()`:

```java
@Override
public void periodic() {
  // 1. Read sensor inputs from hardware
  io.updateInputs(inputs);
  Logger.processInputs("IntakePivot", inputs);

  // 2. Run state machine (only if controlled by SuperStructure)
  if (controlState == ControlState.SUPERSTRUCTURE) {
    updateState();   // Determine internal state from wanted state and sensors
    applyState();    // Send commands to hardware based on internal state
  }

  // 3. Log state for debugging
  Logger.recordOutput("Subsystems/IntakePivot/WantedState", wantedState);
  Logger.recordOutput("Subsystems/IntakePivot/CurrentState", currentState);
  Logger.recordOutput("Subsystems/IntakePivot/ControlState", controlState);
}
```

### Example: Flywheel State Machine

The Flywheel has a complex multi-phase bang-bang controller:

**Wanted States:**
```java
public enum FlywheelWantedStates {
  IDLE,       // Motors off
  SHOOTING,   // Spinup → hold → recover cycle
  COASTING    // Gentle deceleration
}
```

**Internal States:**
```java
public enum FlywheelInternalStates {
  OFF,              // No output
  SPINNING_UP,      // Duty-cycle bang-bang (max accel)
  AT_SETPOINT,      // Torque-current bang-bang (holding)
  RECOVERING,       // Re-spinning after ball fired
  UNDER_SHOOTING,   // Can't recover between rapid shots
  COAST             // Low-power hold
}
```

**State Transition Logic:**

```java
private void updateState() {
  previousState = currentState;

  switch (wantedState) {
    case SHOOTING: {
      double targetRPM = shootVelocitySupplier.getAsDouble();
      boolean atSetpoint = atSetpoint(targetRPM);
      boolean underspeed = isUnderspeed(targetRPM);
      boolean ballFired = (previousState == AT_SETPOINT) && !atSetpoint;

      if (underspeed && previousState == RECOVERING) {
        currentState = UNDER_SHOOTING;
      } else if (ballFired) {
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
    case COASTING:
      currentState = COAST;
      break;
    case IDLE:
    default:
      currentState = OFF;
  }
}

private void applyState() {
  switch (currentState) {
    case SPINNING_UP:
    case RECOVERING:
    case UNDER_SHOOTING:
      setSpinupVelocityControl(shootVelocitySupplier.getAsDouble());
      break;
    case AT_SETPOINT:
      setHoldVelocityControl(shootVelocitySupplier.getAsDouble());
      break;
    case COAST:
      setCoastVelocityControl(shootVelocitySupplier.getAsDouble());
      break;
    case OFF:
    default:
      setDutyCycle(0.0);
  }
}
```

**Key pattern:**
- `updateState()` reads sensors/previous state and transitions to new internal state
- `applyState()` sends hardware commands based on current internal state
- Debounced sensors (`ballFiredDebouncer`, `underspeedDebouncer`) filter noise

---

## Dual Control Modes

Subsystems support **two control modes** to enable both autonomous coordination and manual testing:

```java
public enum ControlState {
  SUPERSTRUCTURE,  // Controlled by SuperStructure state machine
  INDEPENDENT      // Controlled directly by commands
}
```

### SuperStructure Mode

In `SUPERSTRUCTURE` mode, the subsystem's state machine runs every cycle and obeys wanted state transitions from the SuperStructure:

```java
if (controlState == ControlState.SUPERSTRUCTURE) {
  updateState();
  applyState();
}
```

### Independent Mode

In `INDEPENDENT` mode, commands bypass the state machine and drive hardware directly:

```java
// Command factory method
public Command setDutyCycleCommand(double dutyCycle) {
  return runEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0.0));
}
```

**Usage:**

```java
// Switch to independent mode for tuning/testing
superStructure.setControlState(ControlState.INDEPENDENT);

// Command directly controls subsystem
flywheel.setVelocityCommand(3000).schedule();

// Switch back to superstructure mode for competition
superStructure.setControlState(ControlState.SUPERSTRUCTURE);
```

**When SuperStructure sets control state, it propagates to all managed subsystems:**

```java
public void setControlState(ControlState controlState) {
  this.controlState = controlState;
  flywheel.setControlState(controlState);
  indexer.setControlState(controlState);
  intake.setControlState(controlState);
  // ... all other subsystems
}
```

---

## SuperStructure Coordination

### Purpose

The **SuperStructure** coordinates multiple subsystems to execute high-level game actions (intaking, shooting, unjamming, etc.). It **does not own hardware** — it only sets wanted states on subsystems that own hardware through IO layers.

### Architecture

```
SuperStructure (owns state logic)
    ├── Intake (owns IntakeIO)
    ├── Indexer (owns IndexerIO)
    ├── IntakePivot (owns IntakePivotIO)
    ├── HopperRoller (owns HopperRollerIO)
    ├── Flywheel (owns FlywheelIO)
    ├── FlywheelKicker (owns FlywheelKickerIO)
    ├── Hood (owns HoodIO)
    └── ShooterStateMachine (nested state machine)
```

### SuperStructure State Machine

**Wanted States** (set by operator or autonomous):

```java
public enum SuperWantedStates {
  DEFAULT,              // Passive prep (spinup flywheel if in range)
  IDLE,                 // All subsystems off
  INTAKING,             // Collect cargo
  SHOOT_AT_HUB,         // Shoot at speaker
  SHOOT_AT_OUTPOST,     // Pass to outpost
  AUTO_CYCLE_SHOOTING,  // Auto-select hub/outpost by zone
  UNJAMMING,            // Reverse to clear jam
  STOWED                // Safe travel configuration
}
```

**Internal States** (execution phases):

```java
public enum SuperInternalStates {
  DEFAULT,
  IDLE,
  INTAKING,
  SHOOTING_AT_HUB,
  PASSING,
  UNJAMMING,
  STOWING
}
```

### State Application Example

When `INTAKING` is requested, the SuperStructure configures all subsystems:

```java
private void intaking() {
  intake.setWantedState(IntakeStates.INTAKING);
  intakePivot.setWantedState(IntakePivotWantedStates.DEPLOYED);
  indexer.setWantedState(IndexerStates.ASSIST_INTAKING);
  hopperRoller.setWantedState(HopperRollerStates.PREVENT_JAM);
  flywheel.setWantedState(FlywheelWantedStates.IDLE);
  hood.setWantedState(HoodWantedStates.IDLE);
  flywheelKicker.setWantedState(FlywheelKickerStates.OFF);
}
```

When `SHOOTING_AT_HUB` is requested:

```java
private void shooting() {
  intake.setWantedState(IntakeStates.ASSIST_SHOOTING);
  intakePivot.setWantedState(IntakePivotWantedStates.STOWED);
  indexer.setWantedState(IndexerStates.INDEXING);
  hopperRoller.setWantedState(HopperRollerStates.ROLLING);
  
  // Shooter state machine handles multi-phase spinup/fire/recover cycle
  shooterStateMachine.setWantedState(ShooterWantedStates.SHOOTING);
}
```

### Nested State Machines

The SuperStructure contains **nested state machines** for complex multi-subsystem behaviors:

#### ShooterStateMachine

Manages the multi-phase shooting cycle (spinup → ready → firing → recovering):

```java
public enum ShooterWantedStates {
  IDLE,
  SHOOTING
}

public enum ShooterStates {
  IDLE,
  SPINUP,        // Flywheel spinning up
  READY,         // Flywheel at setpoint, hood aimed
  FIRING,        // Kicker feeding ball
  RECOVERING     // Flywheel recovering after shot
}
```

**Lifecycle:**

```
IDLE → SPINUP (flywheel + hood moving)
     → READY (flywheel at setpoint + hood at angle)
     → FIRING (kicker feeds ball, flywheel dips)
     → RECOVERING (flywheel re-spinning)
     → READY (cycle repeats for next ball)
```

The shooter state machine **checks subsystem internal states** to coordinate transitions:

```java
private void update() {
  switch (currentState) {
    case SPINUP:
      if (flywheel.getState() == FlywheelInternalStates.AT_SETPOINT
          && hood.getState() == HoodInternalStates.AT_SETPOINT) {
        currentState = ShooterStates.READY;
      }
      break;
    case READY:
      if (shouldFire.getAsBoolean()) {  // Operator trigger or auto fire
        currentState = ShooterStates.FIRING;
      }
      break;
    case FIRING:
      if (flywheel.getState() == FlywheelInternalStates.RECOVERING) {
        currentState = ShooterStates.RECOVERING;
      }
      break;
    // ...
  }
}
```

#### TargetSelectionStateMachine

Chooses shooting target (hub vs. outpost) based on robot position:

```java
public enum TargetWantedStates {
  AUTO,     // Auto-select based on field zone
  HUB,      // Force hub shot
  OUTPOST   // Force outpost shot
}

public enum TargetInternalStates {
  AT_HUB,
  AT_OUTPOST
}
```

---

## Complete Code Structure

### Typical Subsystem File

```java
package frc.robot.subsystems.HopperRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import org.littletonrobotics.junction.Logger;

public class HopperRoller extends SubsystemBase {
  // ========== CONSTANTS ==========
  private static final double ROLLER_DUTY_CYCLE = 0.75;
  private static final double PREVENT_JAM_DUTY_CYCLE = -0.1;
  private static final double UNJAMMING_DUTY_CYCLE = -0.8;

  // ========== IO LAYER ==========
  private final HopperRollerIO io;
  private final HopperRollerIOInputsAutoLogged inputs = new HopperRollerIOInputsAutoLogged();

  // ========== STATE ENUMS ==========
  public enum HopperRollerStates {
    OFF,
    ROLLING,
    UNJAMMING,
    PREVENT_JAM
  }

  // ========== STATE VARIABLES ==========
  private HopperRollerStates wantedState = HopperRollerStates.OFF;
  private HopperRollerStates currentState = HopperRollerStates.OFF;
  private HopperRollerStates previousState = HopperRollerStates.OFF;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // ========== CONSTRUCTOR ==========
  public HopperRoller(HopperRollerIO io) {
    this.io = io;
  }

  // ========== STATE MACHINE API ==========
  public HopperRollerStates getState() {
    return currentState;
  }

  public void setWantedState(HopperRollerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  // ========== STATE MACHINE LOGIC ==========
  private void updateState() {
    previousState = currentState;
    switch (wantedState) {
      case ROLLING:
        currentState = HopperRollerStates.ROLLING;
        break;
      case PREVENT_JAM:
        currentState = HopperRollerStates.PREVENT_JAM;
        break;
      case UNJAMMING:
        currentState = HopperRollerStates.UNJAMMING;
        break;
      case OFF:
      default:
        currentState = HopperRollerStates.OFF;
    }
  }

  private void applyState() {
    switch (currentState) {
      case ROLLING:
        setDutyCycle(ROLLER_DUTY_CYCLE);
        break;
      case PREVENT_JAM:
        setDutyCycle(PREVENT_JAM_DUTY_CYCLE);
        break;
      case UNJAMMING:
        setDutyCycle(UNJAMMING_DUTY_CYCLE);
        break;
      case OFF:
      default:
        stop();
    }
  }

  // ========== IO DELEGATION ==========
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  // ========== COMMAND FACTORIES ==========
  public Command setDutyCycleCommand(double dutyCycle) {
    return runEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0.0));
  }

  // ========== PERIODIC ==========
  @Override
  public void periodic() {
    // 1. Update inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("HopperRoller", inputs);

    // 2. Run state machine (if controlled by SuperStructure)
    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }

    // 3. Log states
    Logger.recordOutput("Subsystems/HopperRoller/WantedState", wantedState);
    Logger.recordOutput("Subsystems/HopperRoller/CurrentState", currentState);
    Logger.recordOutput("Subsystems/HopperRoller/PreviousState", previousState);
    Logger.recordOutput("Subsystems/HopperRoller/ControlState", controlState);
  }
}
```

### Code Organization Rules

| Section               | Contents                                                                   |
| --------------------- | -------------------------------------------------------------------------- |
| **Constants**         | Tuning values (speeds, tolerances, timeouts) with units in names          |
| **IO Layer**          | `io` reference, `inputs` (AutoLogged)                                      |
| **State Enums**       | `WantedStates`, `InternalStates` (if complex)                              |
| **State Variables**   | `wantedState`, `currentState`, `previousState`, `controlState`             |
| **Constructor**       | Takes `IO` interface, stores reference                                     |
| **State Machine API** | `getState()`, `setWantedState()`, `setControlState()`                      |
| **State Logic**       | `updateState()` (private), `applyState()` (private)                        |
| **IO Delegation**     | Public methods that wrap `io.<method>()`                                   |
| **Command Factories** | `<action>Command()` methods returning `Command` objects                    |
| **Periodic**          | `updateInputs()` → state machine → logging                                 |

---

## Data Flow

### Operational Cycle (20ms loop)

```
┌─────────────────────────────────────────────────────────────────┐
│                     Robot.periodic() (20ms)                      │
└───────────────────────────┬─────────────────────────────────────┘
                            │
        ┌───────────────────┴───────────────────┐
        │                                       │
        ▼                                       ▼
┌──────────────────┐                   ┌──────────────────┐
│  SuperStructure  │                   │  Drive (swerve)  │
│    .periodic()   │                   │    .periodic()   │
└────────┬─────────┘                   └──────────────────┘
         │
         ├─ updateState()  ──→  Set wantedState on all subsystems
         ├─ applyState()   ──→  (subsystems use wanted state to update internal state)
         │
         └─ Each subsystem.periodic():
               │
               ├─ io.updateInputs(inputs)           [READ sensors]
               ├─ Logger.processInputs(...)         [LOG inputs]
               │
               ├─ if (controlState == SUPERSTRUCTURE):
               │     ├─ updateState()                [COMPUTE internal state]
               │     └─ applyState()                 [WRITE to hardware via IO]
               │
               └─ Logger.recordOutput(...)          [LOG states]
```

### Command Data Flow (Independent Mode)

```
Operator Input (Joystick/Button)
       │
       └──→ Command Scheduler
               │
               └──→ someSubsystem.setDutyCycleCommand(0.5)
                       │
                       └──→ subsystem.setDutyCycle(0.5)
                               │
                               └──→ io.setDutyCycle(0.5)
                                       │
                                       └──→ Hardware (motor controller)
```

---

## File Organization

### Directory Structure

```
src/main/java/frc/robot/subsystems/
├── ControlState.java                        # Shared enum
├── SuperStructure.java                      # Coordinator
│
├── Intake/
│   ├── Intake.java                          # Subsystem logic
│   ├── IntakeIO.java                        # IO interface
│   ├── IntakeIOPB.java                      # Practice Bot hardware
│   ├── IntakeIOWB.java                      # WoodBot hardware
│   └── IntakeIOSim.java                     # Simulation
│
├── Indexer/
│   ├── Indexer.java
│   ├── IndexerIO.java
│   ├── IndexerIOPB.java
│   ├── IndexerIOWB.java
│   └── IndexerIOSim.java
│
├── IntakePivot/
│   ├── IntakePivot.java
│   ├── IntakePivotIO.java
│   ├── IntakePivotIOPB.java (SparkMax + cancoder)
│   └── IntakePivotIOSim.java (SingleJointedArmSim)
│
├── HopperRoller/
│   ├── HopperRoller.java
│   ├── HopperRollerIO.java
│   ├── HopperRollerIOPB.java
│   └── HopperRollerIOSim.java
│
├── Shooter/
│   ├── ShotCalculator.java                  # Utility (no IO, no state)
│   ├── ShooterStateMachine.java             # Nested state machine
│   ├── TargetSelectionStateMachine.java     # Nested state machine
│   │
│   ├── Flywheel/
│   │   ├── Flywheel.java
│   │   ├── FlywheelIO.java
│   │   ├── FlywheelIOPB.java (TalonFX bang-bang)
│   │   └── FlywheelIOSim.java
│   │
│   ├── Hood/
│   │   ├── Hood.java
│   │   ├── HoodIO.java
│   │   ├── HoodIOPB.java
│   │   └── HoodIOSim.java
│   │
│   └── FlywheelKicker/
│       ├── FlywheelKicker.java
│       ├── FlywheelKickerIO.java
│       ├── FlywheelKickerIOPB.java
│       └── FlywheelKickerIOSim.java
│
└── Climber/
    ├── Climber.java
    ├── ClimberIO.java
    ├── ClimberIOPB.java
    └── ClimberIOSim.java
```

### Naming Conventions

| Pattern                  | Usage                                                 | Example                          |
| ------------------------ | ----------------------------------------------------- | -------------------------------- |
| `<Subsystem>.java`       | Main subsystem class                                  | `Flywheel.java`                  |
| `<Subsystem>IO.java`     | IO interface                                          | `FlywheelIO.java`                |
| `<Subsystem>IO<Config>`  | Hardware-specific implementation                      | `FlywheelIOPB.java` (PracticeBot)|
| `<Subsystem>IOSim`       | Simulation implementation                             | `FlywheelIOSim.java`             |
| `<Subsystem>WantedStates`| High-level behavior requests                          | `FlywheelWantedStates.SHOOTING`  |
| `<Subsystem>InternalStates` (optional) | Execution phases within wanted behavior | `FlywheelInternalStates.SPINNING_UP` |

---

## Key Principles

### 1. **Single Responsibility**
- **IO layer** = hardware communication only
- **Subsystem** = state logic only
- **SuperStructure** = coordination only

### 2. **Dependency Inversion**
- Subsystems depend on `IO` interface, not concrete hardware implementations
- Enables swapping hardware configs without changing subsystem code

### 3. **Consistent Lifecycle**
All subsystems follow the same pattern:
1. Read inputs (`io.updateInputs()`)
2. Update state (`updateState()`)
3. Apply outputs (`applyState()`)
4. Log everything (`Logger.recordOutput()`)

### 4. **Replay-Based Testing**
- AdvantageKit logs all inputs/outputs
- Can replay logs in simulation to reproduce bugs
- Works because IO layer captures all sensor data

### 5. **Multi-Config Support**
Every robot configuration (SIM, WoodBot, PracticeBot) **must initialize all subsystems**:
- If hardware is missing, use noop IO (interface defaults)
- No null checks needed — SuperStructure always has valid references

---

## Common Patterns

### Pattern: Supplier-Based Dynamic Setpoints

For setpoints that change based on external calculations (e.g., shot calculator):

```java
private DoubleSupplier shootVelocitySupplier = () -> 0.0;

public void setShootVelocitySupplier(DoubleSupplier shootVelocitySupplier) {
  this.shootVelocitySupplier = shootVelocitySupplier;
}

private void applyState() {
  double targetRPM = shootVelocitySupplier.getAsDouble();
  setSpinupVelocityControl(targetRPM);
}
```

**Usage:**

```java
// SuperStructure initialization
flywheel.setShootVelocitySupplier(
    () -> hubShotCalculator.calculateShot().flywheelSpeed());
```

### Pattern: Debounced State Transitions

Use `Debouncer` to filter sensor noise:

```java
private final Debouncer ballFiredDebouncer =
    new Debouncer(0.04, DebounceType.kFalling);

private boolean atSetpoint(double targetRPM) {
  boolean inTolerance = Math.abs(inputs.velocities[0] - targetRPM) < TOLERANCE_RPM;
  return ballFiredDebouncer.calculate(inTolerance);
}
```

### Pattern: State History Tracking

Store previous state to detect transitions:

```java
private void updateState() {
  previousState = currentState;
  
  // Detect edge (transition from one state to another)
  boolean ballFired = (previousState == AT_SETPOINT) && !atSetpoint();
  if (ballFired) {
    launchCount++;
    currentState = RECOVERING;
  }
}
```

### Pattern: Named Constants with Units

```java
private static final double TOLERANCE_DEGREES = 2.0;
private static final double STALL_CURRENT_AMPS = 25.0;
private static final double MAX_VELOCITY_MPS = 4.5;
private static final double SPINUP_TIMEOUT_SECONDS = 3.0;
```

---

## Summary

| Layer               | Responsibility                                   | Files                              |
| ------------------- | ------------------------------------------------ | ---------------------------------- |
| **IO Layer**        | Abstract hardware; enable multi-config/replay    | `*IO.java`, `*IOPB.java`, `*IOSim.java` |
| **Subsystem**       | State machines; own hardware via IO              | `Flywheel.java`, `Intake.java`     |
| **SuperStructure**  | Coordinate subsystems for game actions           | `SuperStructure.java`              |
| **Nested Machines** | Multi-subsystem sequencing                       | `ShooterStateMachine.java`         |

**Benefits:**
- ✅ Same code runs on real robot, simulation, and test fixtures
- ✅ Full replay-based debugging via AdvantageKit
- ✅ Subsystems can be tested independently with commands
- ✅ SuperStructure coordinates complex multi-subsystem behaviors
- ✅ Clear separation of concerns (IO ≠ logic ≠ coordination)
