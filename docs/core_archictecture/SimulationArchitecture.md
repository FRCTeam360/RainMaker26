# FRC Simulation Architecture Guide

This document explains the complete simulation architecture used in our FRC robot code, covering both CTRE Phoenix 6 (TalonFX) and REV Robotics (SparkMax) motor controllers. It demonstrates how to build physics-accurate simulations that share identical control logic with real hardware.

## Summary

The architecture provides:

✅ **Identical control logic** between simulation and real hardware
✅ **Modular design** that scales to complex robots
✅ **Comprehensive logging** via AdvantageKit
✅ **Live tuning** capabilities for rapid iteration
✅ **Realistic constraints** (current limits, battery sag, joint limits)

By following these patterns and best practices, your simulation will be a reliable testing ground for mechanism behavior, PID tuning, and autonomous routines—before the robot is even built.

---
**Target Audience:** This guide assumes familiarity with FRC programming, WPILib, and the AdvantageKit logging framework.

---

## Table of Contents

1. [High-Level Architecture](#1-high-level-architecture)
2. [Core Concepts and Terminology](#2-core-concepts-and-terminology)
3. [The Data Flow Pipeline](#3-the-data-flow-pipeline)
4. [Phoenix 6 Pattern: TalonFX with SingleJointedArmSim](#4-phoenix-6-pattern-talonfx-with-singlejointedarmsim)
5. [REV Robotics Pattern: SparkMax with FlywheelSim](#5-rev-robotics-pattern-sparkmax-with-flywheelsim)
6. [Common Simulation Elements](#6-common-simulation-elements)
8. [Implementation Checklist](#8-implementation-checklist)
9. [Best Practices and Common Pitfalls](#9-best-practices-and-common-pitfalls)

---

## 1. High-Level Architecture

Our codebase follows the **AdvantageKit IO pattern**, which cleanly separates hardware interfaces from subsystem logic:

### Architecture Layers

```
┌─────────────────────────────────────────────────────────┐
│              SUBSYSTEM (IntakePivot, Indexer)           │
│  • Commands and high-level behavior                     │
│  • Owns IO implementation and logged inputs             │
│  • Calls io.updateInputs() each loop                    │
│  • Visualization and telemetry                          │
└─────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────┐
│              IO INTERFACE (*IO)                         │
│  • Defines contract: updateInputs, setPosition, etc.    │
│  • Nested Inputs class (auto-logged by AdvantageKit)    │
└─────────────────────────────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                ▼                       ▼
┌───────────────────────────┐  ┌───────────────────────────┐
│     REAL HARDWARE         │  │      SIMULATION           │
│     (*IOReal)             │  │      (*IOSim)             │
│  • Talks to CAN devices   │  │  • Motor controller sim   │
│  • Reads real sensors     │  │  • WPILib physics models  │
│  • Physical outputs       │  │  • Virtual sensors        │
└───────────────────────────┘  └───────────────────────────┘
```

### Key Principles

- **Single source of truth:** Commands and subsystems remain identical between real robot and simulation
- **Swappable IO:** Change from real to sim by passing a different IO implementation at construction
- **Physics-accurate:** Simulation models match real-world behavior (gravity, inertia, voltage sag)
- **Control-identical:** PID tuning, feedforward, and control modes work the same in sim and real hardware

---

## 2. Core Concepts and Terminology

### Essential Terms

To avoid confusion throughout this document, we use these terms consistently:

| Term | Definition | Code Example |
|------|------------|--------------|
| **Motor Controller** | The device/object that runs closed-loop control and produces output voltage | `TalonFX motorControllerSim`<br>`SparkMax motorControllerSim` |
| **Motor Model** | Mathematical model of an electric motor used by WPILib physics simulation | `DCMotor.getKrakenX60(1)`<br>`DCMotor.getNEO(1)` |
| **Mechanism Physics Model** | WPILib simulator that applies physics (gravity, inertia, limits) to your mechanism | `SingleJointedArmSim`<br>`FlywheelSim`<br>`ElevatorSim` |
| **Control Request** | An object representing a desired control mode and setpoint | `PositionVoltage`<br>`VelocityVoltage`<br>`DutyCycleOut` |

### Why Both Motor Controller AND Motor Model?

This is a common point of confusion. In simulation, you need **both**:

**Motor Controller (TalonFX/SparkMax):**
- Represents the **controller + sensor interface**
- Runs the same internal control logic as real hardware:
  - PID loops (kP, kI, kD)
  - Feedforward (kS, kV, kA, kG)
  - Control modes (position, velocity, duty cycle)
  - Current limits, brake/coast, sensor ratios
- **Computes commanded voltage** from setpoint + sensor feedback
- **Does NOT simulate physics** (no gravity, inertia, or mechanism limits)

**Motor Model (DCMotor):**
- A set of **motor constants** (Kt, Kv, resistance, stall torque, free speed)
- Used by WPILib physics simulation to compute:
  - Torque produced for given voltage and speed
  - Current draw
- **Does NOT run PID or control logic**

**Together they form a closed loop:**
1. Controller: setpoint + sensor feedback → commanded voltage
2. Physics: commanded voltage → new position/velocity/current
3. Feedback: new position/velocity → written back to controller's sensor state

---

## 3. The Data Flow Pipeline

Every simulation follows this data flow each robot loop (20ms):

```
┌────────────────────────────────────────────────────────────────┐
│ 1. COMMAND INPUT                                               │
│    Subsystem/Command calls: setPosition() / setVelocity() /   │
│    setDutyCycle()                                              │
└────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│ 2. MOTOR CONTROLLER COMPUTATION                                │
│    Controller computes voltage based on:                       │
│    • Target (position/velocity/duty cycle)                     │
│    • Measured encoder position/velocity (from sim state)       │
│    • PID + feedforward gains (kP, kD, kG, etc.)               │
│    Output: Commanded voltage                                   │
└────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│ 3. PHYSICS SIMULATION                                          │
│    WPILib mechanism sim takes voltage and computes:            │
│    • New position (considering inertia, gravity, limits)       │
│    • New velocity                                              │
│    • Current draw                                              │
│    • Battery voltage sag                                       │
└────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│ 4. FEEDBACK LOOP CLOSURE                                       │
│    Simulated position/velocity written back to controller's    │
│    sensor state so PID sees movement next cycle                │
└────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│ 5. LOGGING & VISUALIZATION                                     │
│    Inputs struct filled from physics model (source of truth)   │
│    • position, velocity, voltage, current                      │
│    • Logged via AdvantageKit                                   │
│    • Visualizers updated                                       │
└────────────────────────────────────────────────────────────────┘
```

This pipeline ensures that **control logic remains identical** between simulation and real hardware—only the sensor/motor I/O layer changes.

---

## 4. Phoenix 6 Pattern: TalonFX with SingleJointedArmSim

**Example: IntakePivotIOSim**

This section demonstrates simulation of a pivoting intake using CTRE Phoenix 6 motor controllers.

### 4.1. Class Structure

```java
public class IntakePivotIOSim implements IntakePivotIO {
  // Physical constants
  private double gearRatio = 10.0;
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final double armLength = 0.762; // meters (30 inches)
  private final double armMass = 2.0; // kg

  // Control gains
  private final double kG = 0.75; // Gravity compensation

  // Motor controller (Phoenix 6)
  private final TalonFX motorControllerSim =
      new TalonFX(SimulationConstants.INTAKE_PIVOT_MOTOR);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  // Physics simulation
  private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
      gearbox,
      gearRatio,
      SingleJointedArmSim.estimateMOI(armLength, armMass),
      armLength,
      Units.degreesToRadians(-75),  // min angle
      Units.degreesToRadians(255),  // max angle
      true,  // simulate gravity
      0);    // initial angle
}
```

### 4.2. Motor Controller Configuration

Configuration happens once in the constructor via `configureMotor()`:

```java
private void configureMotor() {
  TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  // PID gains for slot 0
  Slot0Configs slot0 = talonConfig.Slot0;
  slot0.kP = 7.0;
  slot0.kI = 0.0;
  slot0.kD = 0.5;
  slot0.kS = 0.0;  // Static friction
  slot0.kV = 0.0;  // Velocity feedforward
  slot0.kA = 0.0;  // Acceleration feedforward
  slot0.kG = 0.75; // Gravity feedforward
  slot0.GravityType = GravityTypeValue.Arm_Cosine; // Cosine compensation

  // Current limits for safety
  CurrentLimitsConfigs currentLimits = talonConfig.CurrentLimits;
  currentLimits.StatorCurrentLimit = 180.0;
  currentLimits.StatorCurrentLimitEnable = true;

  // Brake mode holds position when idle
  talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // Sensor-to-mechanism ratio for cleaner units
  talonConfig.Feedback.SensorToMechanismRatio = gearRatio;

  motorControllerSim.getConfigurator().apply(talonConfig);
}
```

**Key Configuration Elements:**

- **PID + Feedforward:** All control gains live in the motor controller config
- **Gravity Compensation:** `kG` with `Arm_Cosine` automatically handles gravity torque
- **Current Limits:** Protect motor from overcurrent in simulation
- **Sensor Ratio:** Tells TalonFX how encoder rotations map to mechanism rotations

**Critical:** This configuration is **identical** between `*IOSim` and `*IOReal` implementations.

### 4.3. Initialization: Syncing Controller and Physics

The constructor syncs the motor controller's sim state to match the physics model's initial state:

```java
public IntakePivotIOSim() {
  configureMotor();

  // Sync initial state: physics → controller
  motorControllerSim.getSimState().setRawRotorPosition(
      Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
  motorControllerSim.getSimState().setRotorVelocity(
      RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec() * gearRatio)
          .in(RotationsPerSecond));
}
```

**Why sync?**
- The TalonFX PID expects encoder position to match physical mechanism position
- Without sync, PID may see large initial error (or zero error when it shouldn't)
- Prevents unrealistic "jump" on first control command

**Note:** This is optional if both start at zero and are reset consistently, but recommended for robustness.

### 4.4. The updateInputs() Loop (TalonFX)

Called once per robot loop (20ms) by the subsystem:

```java
public void updateInputs(IntakePivotIOInputs inputs) {
  // --- Optional: AdvantageScope tuning (sim-only) ---
  if (tuningEnabled.get()) {
    // Update PID gains on the fly
    Slot0Configs slot0 = new Slot0Configs();
    motorControllerSim.getConfigurator().refresh(slot0);
    slot0.kP = tunableKp.get();
    slot0.kI = tunableKi.get();
    slot0.kD = tunableKd.get();
    motorControllerSim.getConfigurator().apply(slot0);

    // Command tunable setpoint
    motorControllerSim.setControl(
        positionRequest.withPosition(tunableSetpoint.get()));
  }

  // STEP 1: Get commanded voltage from motor controller
  double appliedVoltage = motorControllerSim.getSimState().getMotorVoltage();
  intakePivotSim.setInput(appliedVoltage);

  // STEP 2: Update physics simulation by one timestep (20ms)
  intakePivotSim.update(SimulationConstants.SIM_TICK_RATE_S);

  // STEP 3: Write simulated state back to motor controller
  motorControllerSim.getSimState().setRawRotorPosition(
      Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
  motorControllerSim.getSimState().setRotorVelocity(
      RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec() * gearRatio)
          .in(RotationsPerSecond));

  // STEP 4: Simulate battery voltage sag
  RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
          intakePivotSim.getCurrentDrawAmps()));

  // STEP 5: Fill inputs struct from physics model (source of truth)
  inputs.position = Radians.of(intakePivotSim.getAngleRads()).in(Rotations);
  inputs.velocity = RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec())
      .in(RotationsPerSecond);
  inputs.voltage = appliedVoltage;
  inputs.statorCurrent = intakePivotSim.getCurrentDrawAmps();
  inputs.supplyCurrent = intakePivotSim.getCurrentDrawAmps();
}
```

**Critical Points:**

1. **Voltage source:** Read from `motorControllerSim.getSimState().getMotorVoltage()` (not from applied output)
2. **Physics is truth:** Always read position/velocity/current from `intakePivotSim`, not the motor controller
3. **Feedback loop:** Writing state back to controller lets its PID "see" the mechanism moving
4. **Battery simulation:** Models realistic voltage sag under load

### 4.5. Control Methods (TalonFX)

The IO interface exposes these control modes:

```java
public void setPosition(double positionRotations) {
  motorControllerSim.setControl(positionRequest.withPosition(positionRotations));
}

public void setVelocity(double velocityRPS) {
  motorControllerSim.setControl(velocityRequest.withVelocity(velocityRPS));
}

@Override
public void setDutyCycle(double value) {
  motorControllerSim.set(value);
}
```

**Usage in commands:** Commands call these methods without knowing if they're controlling real hardware or simulation.

---

## 5. REV Robotics Pattern: SparkMax with FlywheelSim

**Example: IndexerIOSim**

This section demonstrates simulation of a flywheel/roller using REV SparkMax motor controllers.

### 5.1. Class Structure

```java
public class IndexerIOSim implements IndexerIO {
  // Physical constants
  private double gearRatio = 5.0;
  private DCMotor gearbox = DCMotor.getNEO(1);
  private final double moi = 0.0513951385; // Moment of inertia in kg·m²

  // Motor controller (REV SparkMax)
  private final SparkMax motorControllerSim =
      new SparkMax(SimulationConstants.INDEXER_MOTOR, MotorType.kBrushless);
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // SparkMax simulation object
  private final SparkMaxSim sparkSim = new SparkMaxSim(motorControllerSim, gearbox);

  // Physics simulation
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, moi, gearRatio);
  private final FlywheelSim indexerSim = new FlywheelSim(plant, gearbox, gearRatio);
}
```

**Key Differences from TalonFX Example:**
- Uses `SparkMax` instead of `TalonFX` (REV-specific)
- Uses `SparkMaxSim` helper object (REV-specific)
- Uses `FlywheelSim` instead of `SingleJointedArmSim` (Non-REV-specific)
- Creates custom `LinearSystem` using `LinearSystemId.createFlywheelSystem()` (Non-REV-specific)

### 5.2. Motor Controller Configuration (SparkMax)

```java
private void configureMotor() {
  motorConfig.idleMode(IdleMode.kBrake);
  motorConfig.inverted(false);
  motorConfig.smartCurrentLimit(40);

  // Apply configuration to motor controller
  motorControllerSim.configure(motorConfig,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kNoPersistParameters);
}
```

**Note:** SparkMax configuration is simpler here because this example uses duty cycle control rather than closed-loop control. For PID control, you would configure PID gains similarly to TalonFX.

### 5.3. The updateInputs() Loop (SparkMax)

The REV pattern differs slightly due to the `SparkMaxSim.iterate()` helper:

```java
public void updateInputs(IndexerIOInputs inputs) {
  // --- Optional: AdvantageScope tuning (sim-only) ---
  if (tuningEnabled.get()) {
    double targetDuty = targetDutyCycle.get();
    motorControllerSim.set(Math.max(-1, Math.min(1, targetDuty)));
  }

  // STEP 1: Get commanded duty cycle and calculate applied voltage
  double commandedDutyCycle = motorControllerSim.get();
  double busVoltage = RoboRioSim.getVInVoltage();
  double appliedVoltage = commandedDutyCycle * busVoltage;

  // STEP 2: Set input voltage to physics simulation
  indexerSim.setInputVoltage(appliedVoltage);

  // STEP 3: Update physics simulation
  indexerSim.update(SimulationConstants.SIM_TICK_RATE_S);

  // STEP 4: Use SparkMaxSim.iterate() to update controller state
  sparkSim.iterate(
      indexerSim.getAngularVelocityRPM(), // Motor velocity in RPM
      RoboRioSim.getVInVoltage(),         // Simulated battery voltage
      SimulationConstants.SIM_TICK_RATE_S);                              // Time interval (20ms)

  // STEP 5: Update battery voltage based on current draw
  RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
          indexerSim.getCurrentDrawAmps()));

  // STEP 6: Fill inputs from physics model (source of truth)
  inputs.position = 0.0; // Position not tracked for flywheel
  inputs.velocity = indexerSim.getAngularVelocityRPM();
  inputs.voltage = appliedVoltage;
  inputs.statorCurrent = indexerSim.getCurrentDrawAmps();
  inputs.supplyCurrent = indexerSim.getCurrentDrawAmps();
  inputs.sensor = false; // Sensor not used in this example
}
```

**Key Differences:**

1. **SparkMaxSim.iterate():** REV provides a helper that updates internal controller state
   - Takes velocity, voltage, and timestep
   - Simpler than manually setting rotor position/velocity

2. **Duty Cycle Control:** This example uses open-loop control (`motorControllerSim.get()`)
   - For closed-loop velocity control, you'd use PID configuration and velocity setpoints

3. **FlywheelSim specifics:**
   - No position tracking (flywheels don't have meaningful position)
   - No angle limits or gravity
   - Only velocity and current draw matter

### 5.4. Control Methods (SparkMax)

```java
@Override
public void setDutyCycle(double value) {
  motorControllerSim.set(value);
}
```

For closed-loop control, you could add:

```java
public void setVelocity(double velocityRPM) {
  // Configure PID controller and use closed-loop control
  motorControllerSim.getPIDController().setReference(
      velocityRPM,
      ControlType.kVelocity);
}
```

---

## 6. Common Simulation Elements

### 6.1. Battery Voltage Simulation

Both patterns simulate realistic battery behavior:

```java
RoboRioSim.setVInVoltage(
    BatterySim.calculateDefaultBatteryLoadedVoltage(
        currentDrawAmps));
```

**Effect:**
- Battery voltage drops when motors draw current
- This voltage drop affects all mechanisms (realistic behavior)
- More current draw → lower voltage → slower acceleration
- Models brownout conditions

### 6.2. AdvantageScope Live Tuning

Both examples include optional real-time tuning (this can also be done for real hardware):

```java
// Define tunable parameters (visible in AdvantageScope)
private final LoggedNetworkNumber tunableKp =
    new LoggedNetworkNumber("/Tuning/IntakePivot/kP", 7.0);
private final LoggedNetworkBoolean tuningEnabled =
    new LoggedNetworkBoolean("/Tuning/IntakePivot/Enabled", false);

// In updateInputs(), apply tuning if enabled
if (tuningEnabled.get()) {
  Slot0Configs slot0 = new Slot0Configs();
  motorControllerSim.getConfigurator().refresh(slot0);
  slot0.kP = tunableKp.get();
  // ... update other gains ...
  motorControllerSim.getConfigurator().apply(slot0);

  motorControllerSim.setControl(
      positionRequest.withPosition(tunableSetpoint.get()));
}
```

**Benefits:**
- Tune PID gains in real-time during simulation
- Test setpoints without rebuilding code
- Values appear under `/Tuning/` table in AdvantageScope

**Note:** This tuning code only exists in `*IOSim`, not in real hardware implementations.

### 6.3. Inputs Struct (Source of Truth)

Both patterns fill the same `Inputs` struct from the **physics model**:

```java
// Always read from physics simulation, not motor controller
inputs.position = /* from physics model */;
inputs.velocity = /* from physics model */;
inputs.voltage = appliedVoltage;
inputs.statorCurrent = /* from physics model */;
inputs.supplyCurrent = /* from physics model */;
```

**Critical:** The motor controller's internal state is a *mirror* used to keep PID happy. The physics model is the authoritative state.


### 6.4. The Right Physics Model

| Mechanism Type | WPILib Class | Key Parameters |
|----------------|--------------|----------------|
| Pivoting Arm/Intake | `SingleJointedArmSim` | Length, mass, min/max angles, gravity |
| Elevator | `ElevatorSim` | Carriage mass, drum radius, min/max height |
| Flywheel/Roller | `FlywheelSim` | Moment of inertia |


---

## 8. Implementation Checklist

When creating a new `*IOSim` class:

### Phase 1: Interface & Structure
- [ ] Reuse existing `*IO` interface or define a new one
- [ ] Define `Inputs` inner class (auto-logged by AdvantageKit)
- [ ] Choose appropriate physics model (`*Sim` class)
- [ ] Select correct `DCMotor` model

### Phase 2: Motor Controller Setup
- [ ] Create motor controller instance (`TalonFX` or `SparkMax`)
- [ ] Implement `configureMotor()` method
- [ ] Configure PID gains (match real hardware config)
- [ ] Set feedforward gains (kS, kV, kA, kG)
- [ ] Configure current limits
- [ ] Set brake/coast mode
- [ ] Set sensor-to-mechanism ratio (if applicable)

### Phase 3: Physics Model Setup
- [ ] Set realistic mass/length/MOI
- [ ] Set correct gear ratio
- [ ] Set motion limits (min/max angle or height)
- [ ] Enable gravity if needed
- [ ] Set initial state

### Phase 4: Constructor
- [ ] Call `configureMotor()`
- [ ] Sync motor controller sim state to physics initial state
  - [ ] Position
  - [ ] Velocity

### Phase 5: updateInputs() Implementation
- [ ] Read commanded voltage from controller
- [ ] Apply voltage to physics model
- [ ] Call `physics.update(SimulationConstants.SIM_TICK_RATE_S)`
- [ ] Write new position/velocity back to controller sim state
- [ ] Update battery voltage via `BatterySim`
- [ ] Fill inputs struct from **physics model** (source of truth)

### Phase 6: Control Methods
- [ ] Implement `setPosition()` (if needed)
- [ ] Implement `setVelocity()` (if needed)
- [ ] Implement `setDutyCycle()` (if needed)

### Phase 7: Subsystem Integration
- [ ] Subsystem owns IO implementation and inputs struct
- [ ] Call `io.updateInputs(inputs)` in `periodic()`
- [ ] Call `Logger.processInputs("SubsystemName", inputs)`
- [ ] Update visualizers from inputs struct

### Phase 8: Testing & Tuning (Optional)
- [ ] Add AdvantageScope tuning parameters
- [ ] Test in simulation with Glass/AdvantageScope
- [ ] Verify physics behavior matches expectations
- [ ] Tune PID gains if using closed-loop control

---

## 9. Best Practices and Common Pitfalls

### 9.1. Units Are Critical

**Common pitfall:** Mixing radians, degrees, and rotations

**Best practice:**
- Physics models use **radians** internally
- Phoenix 6 uses **rotations** by default
- SparkMax can use rotations, RPM, or custom units
- Use WPILib's `Units` class and `edu.wpi.first.units` for conversions

```java
// Good: Explicit unit conversion
Radians.of(angleRads).in(Rotations)
RadiansPerSecond.of(radPerSec).in(RotationsPerSecond)
Units.degreesToRadians(degrees)

// Bad: Manual math without clarity
angleRads * gearRatio / (2 * Math.PI) // What unit is this?
```

### 9.2. Sync State in Both Directions

**Critical:** The feedback loop must be complete

```java
// ✅ CORRECT: Full loop closure
public void updateInputs(...) {
  // Controller → Physics
  voltage = motorController.getSimState().getMotorVoltage();
  physicsSim.setInput(voltage);
  physicsSim.update(SimulationConstants.SIM_TICK_RATE_S);

  // Physics → Controller (CRITICAL!)
  motorController.getSimState().setRawRotorPosition(...);
  motorController.getSimState().setRotorVelocity(...);

  // Physics → Inputs
  inputs.position = physicsSim.getPosition();
}
```

```java
// ❌ WRONG: Missing feedback to controller
public void updateInputs(...) {
  voltage = motorController.getSimState().getMotorVoltage();
  physicsSim.setInput(voltage);
  physicsSim.update(SimulationConstants.SIM_TICK_RATE_S);

  // Missing: Writing state back to controller!
  // Controller's PID won't see mechanism moving

  inputs.position = physicsSim.getPosition();
}
```

### 9.3. Physics Model is Source of Truth

**Always** read sensor values from the physics model, not the motor controller:

```java
// ✅ CORRECT
inputs.position = physicsSim.getAngleRads();
inputs.velocity = physicsSim.getVelocityRadPerSec();
inputs.current = physicsSim.getCurrentDrawAmps();

// ❌ WRONG
inputs.position = motorController.getPosition().getValue();
inputs.velocity = motorController.getVelocity().getValue();
```

The motor controller's values are just mirrors we maintain for PID to work—they're not authoritative.

### 9.4. Keep Configuration Identical

Motor controller configuration should be **byte-for-byte identical** between `*IOSim` and `*IOReal`:

```java
// ✅ CORRECT: Extract to shared method or constants
private void configureMotor() {
  // This exact code appears in both *IOSim and *IOReal
  TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP = 7.0;
  config.Slot0.kD = 0.5;
  config.Slot0.kG = 0.75;
  // ... rest of config ...
  motorController.getConfigurator().apply(config);
}

// ❌ WRONG: Different configs in sim vs real
// IntakePivotIOSim: kP = 5.0
// IntakePivotIOReal: kP = 7.0
// Now sim behavior doesn't match real robot!
```

**Why this matters:** If configs differ, you can't trust simulation results to predict real robot behavior.

### 9.5. Visualize Everything

Keep visualization **out of IO implementations**, in the subsystem:

```java
// ✅ CORRECT: Subsystem owns visualization
public class IntakePivot extends SubsystemBase {
  private final IntakePivotVisualizer visualizer;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Update visualization from logged inputs
    visualizer.update(inputs.position * 2.0 * Math.PI);
  }
}

// ❌ WRONG: Visualization in IO implementation
public class IntakePivotIOSim implements IntakePivotIO {
  private final IntakePivotVisualizer visualizer; // Don't do this!
}
```

**Benefits:**
- Visualization works for both real and sim
- IO stays focused on hardware/physics interaction
- Multiple visualizers can be added without changing IO

### 9.6. Handle Edge Cases

**Implement realistic constraints:**

```java
// Joint limits
private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    gearbox, gearRatio, moi, armLength,
    Units.degreesToRadians(-75),  // Min: hard stop at -75°
    Units.degreesToRadians(255),  // Max: hard stop at 255°
    true, startAngle);

// Current limiting (physics model automatically limits torque)
config.CurrentLimits.StatorCurrentLimit = 180.0;
config.CurrentLimits.StatorCurrentLimitEnable = true;

// Brownout simulation
// Battery voltage automatically drops when drawing current
```

### 9.7. Common TalonFX vs SparkMax Differences

| Aspect | TalonFX (Phoenix 6) | SparkMax (REV) |
|--------|---------------------|----------------|
| **Config object** | `TalonFXConfiguration` | `SparkMaxConfig` |
| **PID slots** | `Slot0Configs`, `Slot1Configs`, etc. | `SparkMaxConfig.closedLoop.pid()` |
| **Control requests** | `PositionVoltage`, `VelocityVoltage` | `getPIDController().setReference()` |
| **Sim state update** | Manual: `getSimState().setRawRotorPosition()` | Helper: `SparkMaxSim.iterate()` |
| **Gravity comp** | Built-in: `kG` + `GravityType` | Manual: Add to feedforward |
| **Units** | Rotations by default | Configurable (rotations, RPM, etc.) |

### 9.8. Testing Your Simulation

**Validation checklist:**

1. **Does it move?**
   - Command a setpoint and verify mechanism moves in Glass/AdvantageScope

2. **Correct direction?**
   - Positive command → expected direction
   - Check motor inversion

3. **Reaches setpoint?**
   - Closed-loop control converges to target

4. **Realistic dynamics?**
   - Check MOI, mass, gear ratio

5. **Respects limits?**
   - Mechanism stops at min/max angles
   - Doesn't go through hard stops

6. **Current draw reasonable?**
   - Check if current limiting works

7. **Battery voltage drops?**
   - High current → voltage sag
   - Multiple mechanisms → more sag

### 9.9. Debugging Simulation Issues

**Problem: Mechanism doesn't move**

Possible causes:
- Not syncing physics state back to controller
- PID gains too low or zero
- Commanded voltage is zero (check control mode)
- Joint at hard limit
- Gear ratio or MOI incorrect (too high)

**Problem: Mechanism moves wrong direction**

Possible causes:
- Motor inverted in config
- Sensor phase incorrect
- Negative gear ratio where positive expected

**Problem: Oscillation or instability**

Possible causes:
- PID gains too high (especially kP or kD)
- Timestep too large (should be SimulationConstants.SIM_TICK_RATE_S)
- Missing (or unexpected) kV or kA terms
- Moment of inertia too low

**Problem: Unrealistic current draw**

Possible causes:
- Incorrect motor model (e.g., using NEO550 when should be Kraken)
- Wrong number of motors in `DCMotor.get*(numMotors)`
- MOI or mass incorrect
- Gear ratio too low or high


---

## 11. Further Resources

### WPILib Documentation
- [Simulation Physics](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)

### Vendor Documentation
- [Phoenix 6 Simulation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/index.html)
- [REVLib Simulation](https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started)

### AdvantageKit Resources
- [AdvantageKit Documentation](https://docs.advantagekit.org/getting-started/what-is-advantagekit/champs-conference)
- [IO Layers](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces)

---

**Document Version:** 1.0
**Last Updated:** January 2026
**Covers:** Phoenix 6 (TalonFX), REV Robotics (SparkMax), WPILib 2026
