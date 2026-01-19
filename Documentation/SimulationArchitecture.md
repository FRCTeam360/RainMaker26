# Simulation Architecture and IO Pattern

This document explains how the simulation code for the intake pivot works and what to watch out for when applying the same structure to other subsystems.

The examples reference `IntakePivotIOSim`, but the same ideas apply to any mechanism (elevator, arm, flywheel, etc.) implemented with this IO + sim pattern.

---

## 1. High‑Level Architecture

The project follows the AdvantageKit IO pattern:

- **Subsystem class** (e.g. `IntakePivot`)
  - Owns commands and high‑level behavior
  - Holds an `IntakePivotIO` implementation and an `IntakePivotIOInputsAutoLogged` struct
  - Calls `io.updateInputs(inputs)` once per loop in `periodic()`
  - Logs the `inputs` struct and may update visualizers

- **IO interface** (e.g. `IntakePivotIO`)
  - Declares methods like `updateInputs`, `setPosition`, `setVelocity`, `setDutyCycle`
  - Has a nested `Inputs` class that is auto‑logged by AdvantageKit

- **IO implementations**
  - **Real**: Talks to real hardware (TalonFX, SparkMAX, etc.)
  - **Sim**: Uses WPILib sim (`SingleJointedArmSim`, `ElevatorSim`, …) and motor controller sim APIs

Only the IO layer changes between real and sim; the subsystem and commands stay identical.

---

## Terminology (read this first)

To avoid confusion, this document uses these terms consistently:

- **Motor controller**: the device/object that runs closed-loop control and produces an output voltage.
  - In this repo: Phoenix 6 `TalonFX` (your `motorControllerSim` field).
- **Motor model**: a *math model* of an electric motor used by WPILib physics simulation.
  - In this repo: `edu.wpi.first.math.system.plant.DCMotor` (e.g. `DCMotor.getKrakenX60(1)`).
- **Mechanism physics model**: the WPILib simulator that applies physics (gravity, inertia, constraints) to your mechanism.
  - In this repo: `SingleJointedArmSim` (your `intakePivotSim` field).

When we say “motor” without qualifiers, we mean the *physical motor* on the real robot. In code, prefer “motor controller” / “motor model” to keep it unambiguous.

---

## 2. Intake Pivot Sim Flow (`IntakePivotIOSim`)

`IntakePivotIOSim` ties together three things:

1. A **motor controller** (Phoenix 6 `TalonFX`)
2. A **mechanism physics model** (`SingleJointedArmSim`)
3. The **AdvantageKit IO inputs struct**

### 2.1. Motor Controller Configuration

Key fields:

```java
private double gearRatio = 10.0;
private DCMotor gearbox = DCMotor.getKrakenX60(1);

private final TalonFX motorControllerSim = new TalonFX(22);
private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
```

Configuration happens once in `configureMotor()`:

```java
TalonFXConfiguration config = new TalonFXConfiguration();

Slot0Configs slot0 = config.Slot0;
slot0.kP = kP;
slot0.kI = kI;
slot0.kD = kD;
slot0.kS = kS;
slot0.kV = kV;
slot0.kA = kA;
slot0.kG = kG;
slot0.GravityType = GravityTypeValue.Arm_Cosine;  // Gravity compensation

CurrentLimitsConfigs currentLimits = config.CurrentLimits;
currentLimits.StatorCurrentLimit = 180.0;
currentLimits.StatorCurrentLimitEnable = true;

config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

config.Feedback.SensorToMechanismRatio = gearRatio;

motorControllerSim.getConfigurator().apply(config);
```

**Key ideas:**

- All PID and gravity compensation live in the **motor controller config**.
- `SensorToMechanismRatio` tells the TalonFX how the encoder maps to mechanism motion.
- This method is **identical** between sim and real hardware IO implementations.

### 2.2. Physics Model (`SingleJointedArmSim`)

```java
private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
  gearbox,              // motor model (Kraken X60)
    gearRatio,            // total reduction
    SingleJointedArmSim.estimateMOI(armLength, armMass),
    armLength,            // meters
    Units.degreesToRadians(-75),  // min angle
    Units.degreesToRadians(255),  // max angle
    true,                 // simulate gravity
    0);                   // initial angle
```

**Responsibilities of the mechanism physics model:**

- Given an **input voltage**, compute:
  - Angular position
  - Angular velocity
  - Current draw
- Enforce joint limits based on min/max angle
- Apply gravity and inertia based on arm mass and length

### 2.3. Initialization

The constructor configures the motor controller and then syncs the motor controller sim state to the physics model:

```java
public IntakePivotIOSim() {
  configureMotor();

  motorControllerSim.getSimState().setRawRotorPosition(
      Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
  motorControllerSim.getSimState().setRotorVelocity(
      RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec() * gearRatio)
          .in(RotationsPerSecond));
}
```

**Why sync the initial state?**

- The TalonFX internal control loops expect the encoder position to match the physical mechanism.
- If the simulated arm starts at one angle and the encoder at another, the PID error is wrong.
- Keeping them in sync avoids a big initial “jump” or no motion due to zero error.

This sync step is useful but not strictly required if you always start at 0 and reset both sides consistently.

---

## 3. The Main Sim Loop: `updateInputs`

`updateInputs` is called once per robot loop by the subsystem. It connects all the pieces:

```java
public void updateInputs(IntakePivotIOInputs inputs) {
  // 1. Get commanded voltage from motor controller
  double appliedVoltage = motorControllerSim.getSimState().getMotorVoltage();
  intakePivotSim.setInput(appliedVoltage);

  // 2. Step the physics simulation by 20 ms
  intakePivotSim.update(0.02);

  // 3. Push the simulated state back into the motor controller sim state
  motorControllerSim.getSimState().setRawRotorPosition(
      Radians.of(intakePivotSim.getAngleRads() * gearRatio).in(Rotations));
  motorControllerSim.getSimState().setRotorVelocity(
      RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec() * gearRatio)
          .in(RotationsPerSecond));

  // 4. Simulate battery voltage sag
  RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
          intakePivotSim.getCurrentDrawAmps()));

  // 5. Fill in AdvantageKit inputs struct from the sim (source of truth)
  inputs.position = Radians.of(intakePivotSim.getAngleRads()).in(Rotations);
  inputs.velocity = RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec())
      .in(RotationsPerSecond);
  inputs.voltage = appliedVoltage;
  inputs.statorCurrent = intakePivotSim.getCurrentDrawAmps();
  inputs.supplyCurrent = intakePivotSim.getCurrentDrawAmps();
}
```

### 3.1. Direction of Data Flow

1. **Commands**: Subsystem or commands call
   - `io.setPosition(targetRotations)`
   - `io.setVelocity(targetRps)`
   - `io.setDutyCycle(percentOutput)`

2. **Motor controller**: TalonFX internally computes a voltage based on
   - Target (position/velocity)
   - Measured encoder position/velocity
   - PID + feedforward (kP, kD, kG, etc.)

3. **Simulation**: `SingleJointedArmSim` takes that voltage and computes
   - New angle, velocity, and current

4. **Feedback**: The TalonFX sim state is updated to match the new physical state, so
   - Next loop, the motor’s PID sees the updated encoder position

5. **Logging and higher‑level logic**: The inputs struct is filled and logged by AdvantageKit.

This structure keeps the **control logic identical** between sim and real hardware; only the sensor/motor I/O changes.

---

## 3.5 TalonFX vs `DCMotor.getKrakenX60(1)` (Why both exist)

A common point of confusion in this pattern is that you’ll see *both* a Phoenix 6 `TalonFX` object and a WPILib `DCMotor` model (like `DCMotor.getKrakenX60(1)`) in the same `*IOSim` class.

They represent two different layers:

### The TalonFX (`new TalonFX(id)`) is the **controller**

The `TalonFX` object represents the **motor controller + its sensor interface**.

- In **real life**, it’s the actual CAN device on the robot.
- In **simulation**, it still runs the same internal control logic based on your config:
  - PID (`kP`, `kI`, `kD`)
  - feedforward terms (`kS`, `kV`, `kA`, `kG`)
  - control modes (`PositionVoltage`, `VelocityVoltage`, duty cycle)
  - brake/coast, current limits, sensor ratio, etc.

In sim, the TalonFX is mainly used for:

- Computing the **commanded motor voltage** from your setpoint + sensor feedback
  - Read via `motorControllerSim.getSimState().getMotorVoltage()`
- Holding the controller’s idea of **sensor position/velocity**
  - Written via `motorControllerSim.getSimState().setRawRotorPosition(...)` and `setRotorVelocity(...)`

**Important:** the TalonFX does *not* simulate mechanism physics (gravity, inertia, arm limits). It only decides what voltage to apply.

### The `DCMotor` Kraken model is the **math model of the motor**

`DCMotor.getKrakenX60(1)` is *not* a controller and it does not run PID.

It’s a set of motor constants (Kt, Kv, resistance, stall torque, free speed, etc.) that WPILib simulation uses to compute:

- torque produced for a given voltage and speed
- current draw

When you pass it into `SingleJointedArmSim`, you’re telling WPILib: “treat the actuator like a Kraken motor (x1 motor) driving this mechanism.”

### How they work together in `updateInputs()`

Each loop you effectively create a closed loop:

1. **Controller layer (TalonFX):** setpoint + (simulated) sensor feedback -> commanded voltage
2. **Physics layer (WPILib sim):** commanded voltage -> new angle/velocity/current
3. **Feedback wiring:** new angle/velocity -> written back into TalonFX sim sensor state

That last feedback step is why we mirror the physics state into `motorControllerSim.getSimState()` every cycle: it lets the TalonFX PID “see” the arm moving, so it can adjust its voltage next cycle.

---

## 4. Control Methods (Position, Velocity, Duty Cycle)

`IntakePivotIOSim` exposes three primary control modes:

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

Because the config is shared between sim and real IO, these methods will behave the same on a real robot.

**Usage in the subsystem** might look like:

```java
public void setPosition(double rotations) {
  io.setPosition(rotations);
}

public void setDutyCycle(double percent) {
  io.setDutyCycle(percent);
}
```

Commands can then target mechanism angles, speeds, or open‑loop outputs without caring about whether it’s simulated or real.

---

## 5. Subsystem Integration and Visualization

The `IntakePivot` subsystem owns the IO instance, the logged inputs struct, and the visualizer:

```java
public class IntakePivot extends SubsystemBase {
  public final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  public final IntakePivotIO io;
  private final IntakePivotVisualizer visualizer;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.visualizer = new IntakePivotVisualizer(0.762); // arm length in meters
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Convert rotations -> radians for visualization
    visualizer.update(inputs.position * 2.0 * Math.PI);
  }
}
```

**Why visualization lives in the subsystem, not IO:**

- Keeps the IO layer focused purely on hardware/sim interaction
- Allows multiple visualizers or different logging strategies
- Works identically for real and sim IO implementations

---

## 6. Applying This Pattern to Other Subsystems

When you adapt this structure to other mechanisms, keep these points in mind.

### 6.1. Choose the Right Physics Model

- **Arm / pivot** → `SingleJointedArmSim`
- **Elevator** → `ElevatorSim`
- **Flywheel** → `FlywheelSim` or custom `LinearSystem`
- **Drivetrain** → `DifferentialDrivetrainSim`, `MecanumDriveSim`, etc.

Match:

- The correct `DCMotor` model (`getFalcon500`, `getKrakenX60`, `getNEO`, …)
- Gearing
- Mass / moment of inertia
- Motion limits (min/max angle or height)

### 6.2. Keep IO Interface Stable

For each subsystem:

- Define a single IO interface (e.g. `ElevatorIO`, `FlywheelIO`)
- Define one `Inputs` struct (e.g. `ElevatorIOInputs`)
- Implement at least two versions:
  - `*IOReal` (talks to hardware)
  - `*IOSim` (uses WPILib sim + controller sim state)

This lets you switch between real and sim at construction time without touching commands.

### 6.3. Sync Controller State with Physics

Whenever you use a motor controller’s internal PID in sim:

- **Initialize** the controller sim state to match the physics model in the constructor
- **Update** the controller sim state from the physics model every loop

Pattern:

```java
// In constructor
motorControllerSim.getSimState().setRawRotorPosition(...fromSim...);
motorControllerSim.getSimState().setRotorVelocity(...fromSim...);

// In updateInputs
sim.update(dt);

motorControllerSim.getSimState().setRawRotorPosition(...fromSim...);
motorControllerSim.getSimState().setRotorVelocity(...fromSim...);
```

If you skip this, the controller PID may:

- See a constant encoder value (no movement)
- Never change its output voltage
- Or think it has already reached the setpoint when it hasn’t

### 6.4. Use the Physics Model as the Source of Truth

Always read positions/velocities/currents from the **mechanism physics model**, not the motor controller:

- ✅ `inputs.position = sim.getPosition...()`
- ✅ `inputs.velocity = sim.getVelocity...()`
- ✅ `inputs.current = sim.getCurrentDrawAmps()`

The motor controller sim state is a mirror used to keep its control loop happy; the mechanism physics model is the authoritative state.

---

## Suggested naming conventions (recommended)

To keep this pattern readable across subsystems, use names like:

- `motorController` / `motorControllerSim`: the Phoenix/REV controller object (`TalonFX`, `SparkMax`, etc.)
- `motorModel`: the WPILib `DCMotor` instance (e.g. Kraken/Falcon/NEO motor constants)
- `mechanismSim`: the WPILib simulation object (`SingleJointedArmSim`, `ElevatorSim`, `FlywheelSim`, ...)

Example:

```java
private final TalonFX motorControllerSim = new TalonFX(22);
private final DCMotor motorModel = DCMotor.getKrakenX60(1);
private final SingleJointedArmSim mechanismSim = new SingleJointedArmSim(...);
```

### 6.5. Keep Logging and Visualization Out of IO

- Subsystems should own:
  - `*IOInputsAutoLogged` structs
  - `Logger.processInputs(...)` calls
  - Visualizers (Mechanism2d/Mechanism3d or AdvantageKit mechanisms)

- IO classes should:
  - Talk to hardware or sim
  - Fill the inputs struct
  - Expose control functions (`setPosition`, `setDutyCycle`, …)

This separation keeps your IO reusable and testable.

### 6.6. Be Careful with Units

Common pitfalls:

- **Radians vs degrees vs rotations**
  - Phoenix 6 position/velocity requests are in *rotations* by default
  - WPILib sim APIs for arms use *radians*
- **Conversions used here:**
  - Radians → rotations: `Radians.of(angleRads).in(Rotations)`
  - Radians/sec → rotations/sec: `RadiansPerSecond.of(radPerSec).in(RotationsPerSecond)`
  - Degrees → radians: `Units.degreesToRadians(deg)`

Pick a consistent internal unit per layer:

- IO sim: physics in radians, motor in rotations
- Subsystem: often uses rotations or degrees
- Commands: choose whatever is most intuitive and convert at the boundaries

---

## 7. Checklist for New Sim IO Implementations

When creating a new `*IOSim`:

1. **Interface**
   - [ ] Reuse existing `*IO` interface or define a new one
   - [ ] Define a single `Inputs` struct and auto‑log it (AdvantageKit)

2. **Motor Controller**
   - [ ] Create motor instance (TalonFX, SparkMAX, etc.)
   - [ ] Configure PID, feedforward, current limits, brake/coast
   - [ ] Set `SensorToMechanismRatio` if applicable

3. **Physics Model**
   - [ ] Pick the appropriate `*Sim` class
   - [ ] Use the correct `DCMotor` model
   - [ ] Set realistic mass, length, gearing, min/max limits

4. **Constructor**
   - [ ] Call a `configureMotor()` helper
   - [ ] Optionally sync controller sim state to physics initial state

5. **updateInputs() Loop**
   - [ ] Read commanded voltage from controller sim state
   - [ ] Apply it to the physics model and call `update(dt)`
   - [ ] Write new position/velocity back into controller sim state
   - [ ] Update battery voltage with `BatterySim`/`RoboRioSim`
   - [ ] Fill the inputs struct from the physics model

6. **Subsystem**
   - [ ] Owns the IO implementation and the inputs struct
   - [ ] Calls `io.updateInputs(inputs)` in `periodic()`
   - [ ] Calls `Logger.processInputs(...)`
   - [ ] Optionally updates one or more visualizers

Following this pattern keeps simulation and real hardware behavior as close as possible while keeping your code modular, testable, and easy to extend to new mechanisms.
