# Alerts Implementation Plan (Issue #545)

## Patterns by Device Type

### SparkFlex — `HopperRoller` ✅ DONE

**IO interface** (`HopperRollerIO.java`):
```java
public boolean motorConnected = true;
```

**Real IO impl** (`HopperRollerIOPB.java`, `HopperRollerIOCB.java`):
```java
import com.revrobotics.spark.SparkLowLevel;

// In updateInputs():
SparkLowLevel.PeriodicStatus0 status0 = hopperRollerMotor.getPeriodicStatus0();
inputs.motorConnected = status0 != null;
```

**Sim/Noop** — set `inputs.motorConnected = true`, no other changes needed.

**Subsystem** (`HopperRoller.java`):
```java
// Field:
private final Alert motorDisconnectedAlert =
    new Alert("Hopper roller motor disconnected", AlertType.kError);

// In periodic():
motorDisconnectedAlert.set(!inputs.motorConnected);
```

---

### TalonFX — all other subsystems

**IO interface**:
```java
// Single motor:
public boolean motorConnected = true;

// Multi-motor (Flywheel, IntakeRoller):
public boolean[] motorsConnected = new boolean[2];
```

**Real IO impl**:
```java
// In updateInputs():
inputs.motorConnected = motor.isConnected();

// Multi-motor:
inputs.motorsConnected[0] = motors[0].isConnected();
inputs.motorsConnected[1] = motors[1].isConnected();
```

**Sim/Noop** — set `inputs.motorConnected = true`, no other changes needed.

**Subsystem**:
```java
// Field:
private final Alert motorDisconnectedAlert =
    new Alert("<SubsystemName> motor disconnected", AlertType.kError);

// In periodic():
motorDisconnectedAlert.set(!inputs.motorConnected);
```

---

### CANrange — `HopperSensor`

`inputs.connected` is already populated in `HopperSensorIOCANRange.java`. Only the subsystem needs updating:

```java
private final Alert sensorDisconnectedAlert =
    new Alert("Hopper CANrange sensor disconnected", AlertType.kError);

// In periodic():
sensorDisconnectedAlert.set(!inputs.connected);
```

---

## Remaining Work

| Subsystem | IO field | Real impls | Subsystem alert |
|---|---|---|---|
| `HopperSensor` | already exists | already populated | ❌ add alert |
| `Hood` | ❌ add `motorConnected` | ❌ `HoodIOPB/CB/WB` | ❌ add alert (1 alert already exists) |
| `Indexer` | ❌ | ❌ `IndexerIOPB/CB/WB` | ❌ |
| `IntakePivot` | ❌ | ❌ `IntakePivotIOPB/CB` | ❌ |
| `IntakeRoller` | ❌ add `motorsConnected[2]` | ❌ `IntakeRollerIOPB/CB/WB` | ❌ two alerts |
| `FlywheelKicker` | ❌ | ❌ `FlywheelKickerIOPB/CB/WB` | ❌ |
| `Flywheel` | ❌ add `motorsConnected[2]` | ❌ `FlywheelIOPB/CB/WB` + bang-bang variants | ❌ two alerts |
