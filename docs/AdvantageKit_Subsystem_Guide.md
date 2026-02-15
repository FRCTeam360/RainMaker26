# AdvantageKit Subsystem Implementation Guide

This guide explains how to create subsystems using the AdvantageKit framework, focusing on the three-layer architecture and the dependency injection pattern that makes it powerful.

---

## Table of Contents

1. [Why AdvantageKit?](#1-why-advantagekit)
2. [The Three-Layer Architecture](#2-the-three-layer-architecture)
3. [Understanding Dependency Injection](#3-understanding-dependency-injection)
4. [Creating a New Subsystem: Step-by-Step](#4-creating-a-new-subsystem-step-by-step)
5. [The IO Layer in Detail](#5-the-io-layer-in-detail)
6. [The Hardware Layer in Detail](#6-the-hardware-layer-in-detail)
7. [The Subsystem Layer in Detail](#7-the-subsystem-layer-in-detail)
8. [Common Patterns and Best Practices](#8-common-patterns-and-best-practices)

---

## 1. Why AdvantageKit?

AdvantageKit is a logging and architecture framework that provides several key benefits:

### Core Benefits

- **Hardware Abstraction**: Swap motor controllers (REV ↔ CTRE) without touching subsystem logic
- **Easy Simulation**: Write physics simulations that use identical subsystem code
- **Comprehensive Logging**: Automatically log all sensor inputs and outputs
- **Testability**: Test subsystem logic without hardware
- **Replay Mode**: Debug issues by replaying logged data (we haven't fully integrated this yet)

### The Key Insight

Traditional FRC code mixes hardware interaction with control logic:

```java
// ❌ Traditional approach - tightly coupled
public class Flywheel extends SubsystemBase {
  private final TalonFX motor = new TalonFX(10);

  public void setVelocity(double velocity) {
    motor.setControl(new VelocityVoltage(velocity));  // Locked to TalonFX
  }
}
```

AdvantageKit separates these concerns:

```java
// ✅ AdvantageKit approach - loosely coupled
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;  // Abstract interface

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);  // Works with ANY implementation
  }
}
```

This separation is achieved through **dependency injection** and a three-layer architecture.

---

## 2. The Three-Layer Architecture

Every AdvantageKit subsystem consists of three layers:

```
┌─────────────────────────────────────────────────────────────────┐
│                      SUBSYSTEM LAYER                            │
│  • High-level behavior and commands                             │
│  • Uses the IO interface (doesn't know which implementation)    │
│  • Calls updateInputs() and processes logs                      │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                        IO LAYER                                 │
│  • Interface defining methods (setVelocity, setPosition, etc.)       │
│  • IOInputs class with @AutoLog annotation                      │
│  • No implementation logic - just contracts                     │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                     HARDWARE LAYER                              │
│  • Multiple implementations of the IO interface                 │
│  • IOReal: talks to actual hardware (TalonFX, SparkMax)         │
│  • IOSim: physics simulation                                    │
│  • IOReplay: plays back logged data                             │
└─────────────────────────────────────────────────────────────────┘
```

### File Structure

For a `Flywheel` subsystem, you'll have:

```
subsystems/
  Flywheel/
    Flywheel.java           ← Subsystem layer
    FlywheelIO.java         ← IO layer (interface + IOInputs)
    FlywheelIOReal.java     ← Hardware layer (real robot)
    FlywheelIOSim.java      ← Hardware layer (simulation)
    FlywheelIOSparkMax.java ← Hardware layer (alternate controller)
```

---

## 3. Understanding Dependency Injection

### What is Dependency Injection?

**Dependency Injection** means passing dependencies to a class rather than creating them inside the class.

```java
// ❌ Without dependency injection
public class Flywheel extends SubsystemBase {
  private final TalonFX motor = new TalonFX(10);  // Created internally
}

// ✅ With dependency injection
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;  // Injected from outside

  public Flywheel(FlywheelIO io) {
    this.io = io;  // Dependency is injected
  }
}
```

### Why Does This Matter?

The subsystem doesn't know or care **which implementation** it receives. You choose at construction time:

```java
// In RobotContainer.java
if (Robot.isReal()) {
  flywheel = new Flywheel(new FlywheelIOReal());
} else if (Robot.isSimulation()) {
  flywheel = new Flywheel(new FlywheelIOSim());
}
```

### The Power of Interfaces

The IO interface defines a **contract**:

```java
public interface FlywheelIO {
  void setVelocity(double velocity);
  void setDutyCycle(double duty);
  void updateInputs(FlywheelIOInputs inputs);
}
```

Every hardware implementation must fulfill this contract:

```java
public class FlywheelIOReal implements FlywheelIO {
  @Override
  public void setVelocity(double velocity) {
    // TalonFX implementation
  }
}

public class FlywheelIOSim implements FlywheelIO {
  @Override
  public void setVelocity(double velocity) {
    // Simulation implementation
  }
}
```

The subsystem calls `io.setVelocity(500)` and doesn't know—or need to know—whether it's talking to a real motor or a simulation.

---

## 4. Creating a New Subsystem: Step-by-Step

Let's create a complete `Indexer` subsystem from scratch.

### Step 1: Create the IO Interface

File: `subsystems/Indexer/IndexerIO.java`

```java
package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  // Define the inputs that will be logged
  @AutoLog
  public static class IndexerIOInputs {
    public double velocity = 0.0;           // Velocity
    public double voltage = 0.0;            // Volts
    public double statorCurrent = 0.0;      // Amps
    public double supplyCurrent = 0.0;      // Amps
  }

  // Define control methods
  public void setDutyCycle(double duty);

  public void setVelocity(double velocity);

  // Update method called every loop
  public default void updateInputs(IndexerIOInputs inputs) {}
}
```

**Key Points:**

- The `@AutoLog` annotation generates `IndexerIOInputsAutoLogged.java` when you build
- Fields in `IOInputs` are what get logged to file
- Methods define the control interface
- `updateInputs()` has a default empty implementation (so you don't need `IOReplay`)

### Step 2: Create the Real Hardware Implementation

File: `subsystems/Indexer/IndexerIOReal.java`

```java
package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {
  private final TalonFX motor;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage velocityRequest = new velocityVoltage(0);

  public IndexerIOReal(int canID) {
    motor = new TalonFX(canID);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID configuration
    config.Slot0.kP = 0.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocity = motor.getVelocity().getValueAsDouble() * 60.0;  // RPS → Velocity
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setDutyCycle(double duty) {
    motor.setControl(dutyCycleRequest.withOutput(duty));
  }

  @Override
  public void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity / 60.0));  // Velocity → RPS
  }
}
```

### Step 3: Create the Simulation Implementation

File: `subsystems/Indexer/IndexerIOSim.java`

```java
package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
  private final FlywheelSim flywheelSim;
  private double appliedVoltage = 0.0;

  public IndexerIOSim() {
    flywheelSim = new FlywheelSim(
        DCMotor.getNEO(1),     // Motor model
        5.0,                    // Gear ratio
        0.05);                  // Moment of inertia (kg·m²)
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    flywheelSim.update(0.02);  // Advance simulation by 20ms

    inputs.velocity = flywheelSim.getAngularVelocityRPM();
    inputs.voltage = appliedVoltage;
    inputs.statorCurrent = flywheelSim.getCurrentDrawAmps();
    inputs.supplyCurrent = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setDutyCycle(double duty) {
    appliedVoltage = duty * 12.0;  // Assume 12V battery
    flywheelSim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocity(double velocity) {
    // Simple approximation: use voltage to reach target Velocity
    // In reality, you'd implement PID in simulation
    double targetVoltage = velocity / 500.0;  // Example scaling
    appliedVoltage = targetVoltage;
    flywheelSim.setInputVoltage(appliedVoltage);
  }
}
```

### Step 4: Create the Subsystem

File: `subsystems/Indexer/Indexer.java`

```java
package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer subsystem */
  public Indexer(IndexerIO io) {
    this.io = io;  // Dependency injection!
  }

  @Override
  public void periodic() {
    // Update inputs from hardware/sim
    io.updateInputs(inputs);

    // Process and log the inputs
    Logger.processInputs("Indexer", inputs);
  }

  // High-level control methods
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  // Command factories
  public Command runAtVelocity(double velocity) {
    return this.runEnd(() -> setVelocity(velocity), this::stop);
  }
}
```

### Step 5: Wire It Up in RobotContainer

File: `RobotContainer.java`

```java
public class RobotContainer {
  private final Indexer indexer;

  public RobotContainer() {
    // Choose implementation based on robot mode
    if (Robot.isReal()) {
      indexer = new Indexer(new IndexerIOReal(10));  // CAN ID 10
    } else {
      indexer = new Indexer(new IndexerIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    // Example: Run indexer when A button is pressed
    controller.a().whileTrue(indexer.runAtVelocity(3000));
  }
}
```

---

## 5. The IO Layer in Detail

### Purpose

The IO layer defines the **contract** between subsystem logic and hardware interaction.

### Components

#### 5.1 The Interface

Defines methods that all hardware implementations must provide:

```java
public interface FlywheelIO {
  void setVelocity(double velocity);
  void setDutyCycle(double duty);
  void updateInputs(FlywheelIOInputs inputs);
}
```

**Rules:**

- Keep methods simple and high-level
- Use units consistently (document them in comments)
- Provide default implementations when appropriate

```java
public interface FlywheelIO {
  /** Set flywheel velocity in Velocity */
  void setVelocity(double velocity);

  /** Optional: stop the flywheel (default implementation) */
  default void stop() {
    setDutyCycle(0.0);
  }

  void updateInputs(FlywheelIOInputs inputs);
}
```

#### 5.2 The IOInputs Class

Defines what data gets logged and passed to the subsystem:

```java
@AutoLog
public static class FlywheelIOInputs {
  public double velocity = 0.0;        // Velocity
  public double voltage = 0.0;         // Volts
  public double statorCurrent = 0.0;   // Amps
  public double temperature = 0.0;     // Celsius
}
```

**The @AutoLog Annotation:**

When you build your code, AdvantageKit generates `FlywheelIOInputsAutoLogged.java`:

```
build/
  generated/
    sources/
      annotationProcessor/
        java/
          main/
            frc/
              robot/
                subsystems/
                  Flywheel/
                    FlywheelIOInputsAutoLogged.java  ← Generated file
```

**What it generates:**

- `toLog(LogTable)`: writes fields to the log
- `fromLog(LogTable)`: reads fields from the log (for replay)
- Automatic handling of arrays and nested structures

**Best Practices for IOInputs:**

1. **Use clear names and units:**
   ```java
   public double velocityRPM = 0.0;      // ✅ Clear
   public double vel = 0.0;              // ❌ Ambiguous
   ```

2. **Initialize to sensible defaults:**
   ```java
   public double voltage = 0.0;          // ✅ Safe default
   public boolean isConnected = false;   // ✅ Safe default
   ```

3. **Use arrays for multiple similar sensors:**
   ```java
   public double[] motorVoltages = new double[2];
   public double[] motorCurrents = new double[2];
   ```

4. **Group related data:**
   ```java
   public double position = 0.0;
   public double velocity = 0.0;
   public double acceleration = 0.0;
   ```

---

## 6. The Hardware Layer in Detail

### Purpose

Implements the IO interface for specific hardware or simulation.

### Common Implementations

#### 6.1 Real Hardware (TalonFX Example)

```java
public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new velocityVoltage(0);

  public FlywheelIOReal(int canID) {
    motor = new TalonFX(canID);
    configureMotor();
  }

  private void configureMotor() {
    // Configuration logic
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.5;
    // ... more config
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Read from hardware
    inputs.velocity = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity / 60.0));
  }
}
```

#### 6.2 Alternative Hardware (SparkMax Example)

```java
public class FlywheelIOSparkMax implements FlywheelIO {
  private final SparkMax motor;
  private final SparkClosedLoopController controller;

  public FlywheelIOSparkMax(int canID) {
    motor = new SparkMax(canID, MotorType.kBrushless);
    controller = motor.getClosedLoopController();
    configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(0.0005, 0, 0);
    // ... more config
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocity = motor.getEncoder().getVelocity();  // Already in RPM
    inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.statorCurrent = motor.getOutputCurrent();
  }

  @Override
  public void setVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }
}
```

**The Key Point:** The subsystem doesn't change at all—only the hardware implementation changes.

#### 6.3 Simulation

See Step 3 in the previous section for a complete simulation example. Key elements:

- Use WPILib physics models (`FlywheelSim`, `SingleJointedArmSim`, etc.)
- Advance simulation in `updateInputs()` with `sim.update(0.02)`
- Maintain internal state (applied voltage, position, etc.)

---

## 7. The Subsystem Layer in Detail

### Purpose

Contains all high-level behavior, commands, and logic. **No hardware knowledge.**

### Structure

```java
public class Flywheel extends SubsystemBase {
  // IO interface (dependency injection)
  private final FlywheelIO io;

  // Auto-logged inputs
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Additional state (not logged through IO)
  private double targetVelocity = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // 1. Update inputs from hardware/sim
    io.updateInputs(inputs);

    // 2. Process and log inputs
    Logger.processInputs("Flywheel", inputs);

    // 3. Log additional state
    Logger.recordOutput("Flywheel/TargetVelocity", targetVelocity);

    // 4. Any periodic logic (state machines, etc.)
  }

  // Control methods
  public void setVelocity(double velocity) {
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  // Queries
  public double getVelocity() {
    return inputs.velocity;
  }

  public boolean atTarget() {
    return Math.abs(inputs.velocity - targetVelocity) < 50.0;  // Within 50 Velocity
  }

  // Command factories
  public Command spinUpCommand(double velocity) {
    return this.runEnd(
        () -> setVelocity(velocity),
        () -> setVelocity(0))
        .withName("SpinUp");
  }
}
```

### Best Practices

1. **Keep subsystem hardware-agnostic**
   ```java
   // ✅ Good - uses IO interface
   io.setVelocity(velocity);

   // ❌ Bad - knows about specific hardware
   motor.setControl(new velocityVoltage(velocity));
   ```

2. **Log everything important**
   ```java
   Logger.recordOutput("Flywheel/AtTarget", atTarget());
   Logger.recordOutput("Flywheel/Error", targetVelocity - inputs.velocity);
   ```

3. **Create command factories**
   ```java
   public Command shootCommand() {
     return Commands.sequence(
         spinUpCommand(5000),
         Commands.waitUntil(this::atTarget),
         feedCommand()
     );
   }
   ```

---

## 8. Common Patterns and Best Practices

### 8.1 Multiple Motors

When you have multiple motors in one subsystem:

```java
@AutoLog
public static class FlywheelIOInputs {
  public static final int NUM_MOTORS = 2;

  public double[] velocities = new double[NUM_MOTORS];
  public double[] voltages = new double[NUM_MOTORS];
  public double[] currents = new double[NUM_MOTORS];
}
```

Hardware implementation:

```java
public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX leader;
  private final TalonFX follower;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocities[0] = leader.getVelocity().getValueAsDouble() * 60.0;
    inputs.velocities[1] = follower.getVelocity().getValueAsDouble() * 60.0;
    // ... etc
  }
}
```

### 8.2 Using Enums for States

```java
public class Indexer extends SubsystemBase {
  public enum State {
    IDLE,
    INTAKING,
    FEEDING,
    REVERSING
  }

  private State currentState = State.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/State", currentState.toString());

    // State machine logic
    switch (currentState) {
      case IDLE:
        io.setDutyCycle(0.0);
        break;
      case INTAKING:
        io.setDutyCycle(0.5);
        break;
      // ... etc
    }
  }

  public void setState(State state) {
    currentState = state;
  }
}
```

### 8.3 Optional Hardware

Use default methods in the interface:

```java
public interface ArmIO {
  void setPosition(double position);

  // Optional absolute encoder
  default double getAbsolutePosition() {
    return 0.0;  // Default: not supported
  }

  void updateInputs(ArmIOInputs inputs);
}
```

Then in the hardware layer:

```java
public class ArmIOReal implements ArmIO {
  private final CANcoder absoluteEncoder;  // Optional

  @Override
  public double getAbsolutePosition() {
    if (absoluteEncoder != null) {
      return absoluteEncoder.getAbsolutePosition().getValue();
    }
    return super.getAbsolutePosition();  // Fall back to default
  }
}
```

### 8.4 Unit Conversions

Keep units consistent within each layer:

- **IO Interface**: Use mechanism units (rotations, velocity, degrees)
- **Hardware Layer**: Convert between mechanism and motor units
- **Subsystem Layer**: Use mechanism units

```java
public class ElevatorIOReal implements ElevatorIO {
  private static final double SPOOL_RADIUS = 0.02;  // meters
  private static final double GEAR_RATIO = 10.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Motor rotations → mechanism height
    double motorRotations = motor.getPosition().getValueAsDouble();
    double spoolRotations = motorRotations / GEAR_RATIO;
    inputs.heightMeters = spoolRotations * 2 * Math.PI * SPOOL_RADIUS;
  }

  @Override
  public void setHeight(double meters) {
    // Mechanism height → motor rotations
    double spoolRotations = meters / (2 * Math.PI * SPOOL_RADIUS);
    double motorRotations = spoolRotations * GEAR_RATIO;
    motor.setControl(positionRequest.withPosition(motorRotations));
  }
}
```

### 8.5 Checklist for New Subsystems

- [ ] Create IO interface with `@AutoLog` inputs class
- [ ] Define all control methods in the interface
- [ ] Create at least one hardware implementation (`IOReal` or `IOSim`)
- [ ] Implement `updateInputs()` in hardware layer
- [ ] Create subsystem class that takes `IO` interface in constructor
- [ ] Call `io.updateInputs(inputs)` in subsystem `periodic()`
- [ ] Call `Logger.processInputs()` after updating inputs
- [ ] Wire up subsystem in `RobotContainer` with appropriate IO implementation
- [ ] Test in simulation before deploying to robot

---

## Summary

The AdvantageKit IO pattern provides:

1. **Separation of Concerns**: Hardware interaction is isolated from control logic
2. **Dependency Injection**: Subsystems receive IO implementations, enabling swappable hardware
3. **Comprehensive Logging**: `@AutoLog` automatically logs all inputs
4. **Easy Testing**: Mock IO implementations for unit tests
5. **Simulation Support**: Write physics sims without changing subsystem code

The pattern takes a bit more boilerplate upfront, but pays huge dividends in code quality, testability, and flexibility.
