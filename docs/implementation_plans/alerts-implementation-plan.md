# Alerts Implementation Plan (Issue #545)

## Goal

Add hardware health alerts to every motor-based and sensor subsystem so the drive team gets early warning of failing devices during matches and diagnostics.

The two pillars from the issue:
1. **Device connectivity** — detect when a device stops communicating on CAN
2. **Heartbeat monitoring** — per-device "is this alive?" signal surfaced to the DS

---

## WPILib Alert API (existing pattern from `Hood.java`)

```java
// Declaration (in subsystem class)
private final Alert myAlert = new Alert("Message shown on DS", AlertType.kWarning);

// In periodic()
myAlert.set(conditionIsTrue);
```

Alerts appear on the Driver Station and in AdvantageScope. Use `AlertType.kError` for connectivity loss and stalls, `AlertType.kWarning` for temperature and non-fatal device warnings.

---

## Hardware API Reference

### TalonFX (CTRE Phoenix 6)
Used by: Flywheel (×2), FlywheelKicker, Hood, Indexer, IntakePivot, IntakeRoller (×2)

#### Connectivity check — `isConnected()`

`TalonFX` inherits `isConnected()` from `ParentDevice` — this is the direct equivalent to what the user described. It works by refreshing the device's Version status signal and checking its latency:

- `motor.isConnected()` — uses a default 0.5 s latency window
- `motor.isConnected(double maxLatencySeconds)` — configurable timeout for stricter/looser detection

```java
// In updateInputs():
inputs.motorConnected = motor.isConnected();
```

For multi-motor subsystems (e.g. Flywheel):

```java
inputs.motorsConnected[0] = motors[0].isConnected();
inputs.motorsConnected[1] = motors[1].isConnected();
```

Note: `isConnected()` internally refreshes the Version signal, so it does add a small amount of CAN traffic per call. At 50 Hz this is negligible. The `BaseStatusSignal.refreshAll().isOK()` pattern (used in `HopperSensorIOCANRange.java:54–57`) is an alternative that piggybacks on already-scheduled signal refreshes, but `isConnected()` is the cleaner, purpose-built API.

#### Temperature signal

`TalonFX.getDeviceTemp()` returns a `StatusSignal<Temperature>`. Register it alongside existing signals in the constructor; set its update frequency low (4 Hz) since temperature changes slowly:

```java
// In constructor:
private final StatusSignal<Temperature> deviceTempSignal = motor.getDeviceTemp();
// ...
deviceTempSignal.setUpdateFrequency(4);
motor.optimizeBusUtilization();

// In updateInputs():
deviceTempSignal.refresh();
inputs.motorTemperatureCelsius = deviceTempSignal.getValueAsDouble();
```

#### Stall detection

No new signals needed — `statorCurrent` and `velocity` are already logged. Stall is detected in the subsystem layer:

```java
// In subsystem periodic():
boolean isStalled = inputs.statorCurrent > STALL_CURRENT_THRESHOLD_AMPS
    && Math.abs(inputs.velocity) < STALL_VELOCITY_THRESHOLD;
stallAlert.set(isStalled);
```

---

### SparkFlex (REV)
Used by: HopperRoller

#### Connectivity check — `getPeriodicStatus0()` + `canFault`

`SparkLowLevel` exposes `getPeriodicStatus0()` through `getPeriodicStatus9()`, all returning `@Nullable` status objects. **A null return means the frame was never received** — the device is not communicating on CAN. This is the closest REV equivalent to TalonFX's `isConnected()`:

```java
// In updateInputs():
SparkLowLevel.PeriodicStatus0 status0 = hopperRollerMotor.getPeriodicStatus0();
inputs.motorConnected = status0 != null;
```

`PeriodicStatus1` carries a `canFault` boolean directly — this is a CAN-specific fault flag reported by the device itself. Use it as a secondary check for CAN-layer errors when the device is communicating but experiencing CAN issues:

```java
SparkLowLevel.PeriodicStatus1 status1 = hopperRollerMotor.getPeriodicStatus1();
// guard null in case device is fully disconnected
boolean canFault = status1 != null && status1.canFault;
inputs.motorConnected = status0 != null && !canFault;
```

`PeriodicStatus0` also has a `primaryHeartbeatLock` boolean — this is **not** a connectivity indicator. It reflects whether the roboRIO is actively sending DS heartbeat packets to the SPARK (goes false when robot is disabled or DS loses link). Do not use it for hardware fault detection.

#### Fault detection — `hasActiveFault()` / `hasActiveWarning()`

For non-connectivity hardware faults (temperature, gate driver, sensor, etc.):

```java
inputs.motorFaulted = hopperRollerMotor.hasActiveFault();
inputs.motorWarned  = hopperRollerMotor.hasActiveWarning();
```

`getLastError()` only reflects the result of the last single operation — do not use it for persistent connectivity monitoring.

---

### CANrange (CTRE)
Used by: HopperSensor

`connected` field already exists in `HopperSensorIOCANRange.java` using the same `statusSignal.getStatus().isOK()` pattern as TalonFX. No IO changes needed — just add the Alert in `HopperSensor.java`.

---

## Alert Types & Conditions

### TalonFX subsystems

| Alert | Detection Method |
|-------|-----------------|
| Device disconnected | `!inputs.motorConnected` (from `refreshAll` status) |
| Motor stall | `inputs.statorCurrent > THRESHOLD && Math.abs(inputs.velocity) < THRESHOLD` |
| Temperature warning | `inputs.motorTemperatureCelsius > TEMP_WARNING_THRESHOLD_CELSIUS` |

### SparkFlex (HopperRoller)

| Alert | Detection Method |
|-------|-----------------|
| Device disconnected | `!inputs.motorConnected` (from `getPeriodicStatus0() == null`) |
| Device fault | `inputs.motorFaulted` (from `hasActiveFault()`) |
| Device warning | `inputs.motorWarned` (from `hasActiveWarning()`) |

### CANrange (HopperSensor)

| Alert | Detection Method |
|-------|-----------------|
| Sensor disconnected | `!inputs.connected` (already populated) |

---

## Implementation Plan

### Step 1 — Add connectivity fields to all IO inputs

For each TalonFX-based IO implementation, capture the `StatusCode` from `BaseStatusSignal.refreshAll()` and write a `boolean motorConnected` (or `boolean[] motorsConnected` for multi-motor subsystems) field into the inputs class.

**Files to change:**

| IO Implementation | Inputs field to add | Notes |
|---|---|---|
| `FlywheelIO.java` | `boolean[] motorsConnected = new boolean[2]` | Two TalonFXs |
| `FlywheelIOPB.java` / `FlywheelIOCB.java` / `FlywheelIOWB.java` | Capture `StatusCode` from `refreshAll` | Repeat for all real impls |
| `FlywheelKickerIO.java` | `boolean motorConnected` | Single TalonFX |
| `FlywheelKickerIOPB/CB/WB.java` | Same pattern | |
| `HoodIO.java` | `boolean motorConnected` | Single TalonFX |
| `HoodIOPB/CB/WB.java` | Same pattern | |
| `IndexerIO.java` | `boolean motorConnected` | Single TalonFX |
| `IndexerIOPB/CB/WB.java` | Same pattern | |
| `IntakePivotIO.java` | `boolean motorConnected` | Single TalonFX |
| `IntakePivotIOPB/CB.java` | Same pattern | |
| `IntakeRollerIO.java` | `boolean[] motorsConnected = new boolean[2]` | Two TalonFXs |
| `IntakeRollerIOPB/CB/WB.java` | Same pattern | |
| `HopperRollerIO.java` | `boolean motorFaulted`, `boolean motorWarned` | SparkFlex |
| `HopperRollerIOPB/CB.java` | Use `motor.getFaults()` / `motor.getWarnings()` | |

See the **Hardware API Reference** section above for exact call patterns. **Noop/Sim implementations:** Set `motorConnected = true`, `motorFaulted = false` so alerts never fire outside real hardware configs.

---

### Step 2 — Add temperature signals to TalonFX IO inputs (optional but recommended)

Add `double motorTemperatureCelsius` (or `double[] motorTemperaturesCelsius` for multi-motor) to the IO inputs of each TalonFX subsystem. Wire up `motor.getDeviceTemp()` as a StatusSignal alongside the existing signals. Set update frequency to 4 Hz (temperature changes slowly).

```java
private final StatusSignal<Temperature> deviceTempSignal = motors[0].getDeviceTemp();
// In constructor:
deviceTempSignal.setUpdateFrequency(4);
// In updateInputs:
inputs.motorTemperatureCelsius = deviceTempSignal.getValueAsDouble();
```

---

### Step 3 — Add alerts to each subsystem class

All alerts live in the subsystem (not the IO layer), reading from `inputs` in `periodic()`. Follow the existing Hood.java pattern exactly.

**Per-subsystem alert declarations and conditions:**

#### `Flywheel.java`
```java
private final Alert rightMotorDisconnectedAlert = new Alert("Flywheel right motor disconnected", AlertType.kError);
private final Alert leftMotorDisconnectedAlert  = new Alert("Flywheel left motor disconnected",  AlertType.kError);
private final Alert flywheelOverTempAlert       = new Alert("Flywheel motor over temperature",   AlertType.kWarning);
```
Conditions:
- `!inputs.motorsConnected[0]`, `!inputs.motorsConnected[1]`
- `inputs.motorTemperaturesCelsius[0] > TEMP_WARNING_THRESHOLD_CELSIUS` (suggested: 70°C)

#### `FlywheelKicker.java`
```java
private final Alert kickerMotorDisconnectedAlert = new Alert("Flywheel kicker motor disconnected", AlertType.kError);
```

#### `Hood.java` (already has one alert — extend it)
```java
private final Alert hoodMotorDisconnectedAlert = new Alert("Hood motor disconnected", AlertType.kError);
private final Alert hoodStallAlert             = new Alert("Hood motor stalling",     AlertType.kWarning);
```
Stall condition: `inputs.statorCurrent > STALL_CURRENT_THRESHOLD_AMPS && Math.abs(inputs.velocity) < STALL_VELOCITY_THRESHOLD_DEG_PER_SEC`

#### `Indexer.java`
```java
private final Alert indexerMotorDisconnectedAlert = new Alert("Indexer motor disconnected", AlertType.kError);
private final Alert indexerStallAlert             = new Alert("Indexer motor stalling",     AlertType.kWarning);
```

#### `IntakePivot.java`
```java
private final Alert pivotMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected", AlertType.kError);
private final Alert pivotStallAlert             = new Alert("Intake pivot motor stalling",     AlertType.kWarning);
```

#### `IntakeRoller.java`
```java
private final Alert leftRollerDisconnectedAlert  = new Alert("Intake left roller motor disconnected",  AlertType.kError);
private final Alert rightRollerDisconnectedAlert = new Alert("Intake right roller motor disconnected", AlertType.kError);
```

#### `HopperRoller.java`
```java
private final Alert hopperMotorFaultAlert   = new Alert("Hopper roller motor fault",   AlertType.kError);
private final Alert hopperMotorWarningAlert = new Alert("Hopper roller motor warning", AlertType.kWarning);
```

#### `HopperSensor.java`
```java
private final Alert hopperSensorDisconnectedAlert = new Alert("Hopper CANrange sensor disconnected", AlertType.kError);
```
Use existing `inputs.connected` field — no IO changes needed.

---

## Stall Thresholds (starting values — tune on robot)

| Subsystem | Stall Current (A) | Stall Velocity |
|---|---|---|
| Flywheel | N/A (open-ended spin-up, stall unlikely) | — |
| Hood | 15 A stator | < 5 deg/s |
| Indexer | 30 A stator | < 5 RPM |
| IntakePivot | 20 A stator | < 5 deg/s |
| IntakeRoller | 25 A stator (per motor) | < 30 RPM |
| HopperRoller | 15 A stator | < 30 RPM |

Temperature warning threshold: **70°C** for all TalonFX motors.

---

## File Change Summary

| File | Change |
|---|---|
| `*IO.java` (all motor IO interfaces) | Add `motorConnected` / `motorsConnected[]` + optional temp fields |
| `*IOPB/CB/WB.java` (all real impls) | Capture `StatusCode` from `refreshAll`, add temp signal |
| `*IONoop.java` (all noop impls) | Set `motorConnected = true`, `motorFaulted = false` |
| `*IOSim.java` (all sim impls) | Set `motorConnected = true` |
| `HopperRollerIOPB/CB.java` | Add `getFaults()` / `getWarnings()` reads |
| `Flywheel.java` | Add 3 alerts |
| `FlywheelKicker.java` | Add 1 alert |
| `Hood.java` | Add 2 alerts (1 already exists) |
| `Indexer.java` | Add 2 alerts |
| `IntakePivot.java` | Add 2 alerts |
| `IntakeRoller.java` | Add 2 alerts |
| `HopperRoller.java` | Add 2 alerts |
| `HopperSensor.java` | Add 1 alert (use existing `connected` field) |

---

## Implementation Order

1. `HopperSensor.java` — easiest, `connected` field already exists, just add the Alert
2. `Hood.java` — add connectivity + stall alerts alongside the existing hood-up alert (good reference point)
3. All other TalonFX subsystems — mechanical work, same pattern repeated
4. `HopperRoller.java` — SparkFlex pattern is slightly different
5. Temperature signals — add after all connectivity alerts are validated on robot
