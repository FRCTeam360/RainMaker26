# Bang-Bang Flywheel Control for High-Volume Shooters

## Overview

This document explains the bang-bang flywheel control strategy used in our shooter, why it exists, how it differs from traditional PID, and the key implementation details that make it work for rapid-fire shooting.

## The Problem with PID for Rapid Fire

A traditional PID velocity controller applies a **proportional** output based on error. When a ball passes through the flywheel:

1. Velocity drops suddenly (e.g. 3500 RPM → 3200 RPM)
2. PID sees a 300 RPM error and applies a moderate correction
3. The flywheel gradually recovers over several hundred milliseconds
4. The next ball can't fire until velocity is back in tolerance

For a single-shot shooter, this is fine. For rapid fire (multiple balls per second), the recovery time between shots is the bottleneck. PID's proportional response means it never applies more output than the error demands — it **ramps up** to the correction instead of slamming full power immediately.

## How Bang-Bang Solves This

Bang-bang control is binary: the motor is either **full on** or **off** (or in our case, switches between two control modes). There is no proportional region. The moment velocity drops below the setpoint, the motor goes to 100% output and stays there until velocity is restored.

This gives the **fastest possible recovery time** — the motor is always applying maximum torque to get back to speed.

## Our Two-Phase Implementation

We use a **two-phase bang-bang** approach rather than a simple on/off:

### Phase 1: Spin-Up (Duty Cycle Bang-Bang)

- **Control mode:** `VelocityDutyCycle` with Slot 0 (`kP = 999999`)
- **Behavior:** Below setpoint → full duty cycle output (100%). Above → zero.
- **When used:** Flywheel state is `SPINNING_UP` (velocity below tolerance)
- **Purpose:** Maximum acceleration with no current limiting. Gets the flywheel to speed as fast as physically possible.

The extremely high `kP` with all other gains at zero turns Phoenix 6's velocity controller into a relay — any negative error produces output that saturates at `PeakForwardDutyCycle` (1.0), and any positive error produces zero.

### Phase 2: Hold (Torque Current Bang-Bang)

- **Control mode:** `VelocityTorqueCurrentFOC` with Slot 0 (`kP = 999999`)
- **Behavior:** Same bang-bang logic, but output is **torque current** limited to 30A
- **When used:** Flywheel state is `AT_SETPOINT` (velocity within tolerance)
- **Purpose:** Maintains velocity with consistent, bounded torque. Prevents the flywheel from overshooting wildly and limits current draw while at speed.

The `PeakForwardTorqueCurrent = 30A` cap is the key difference from Phase 1. At setpoint, the flywheel only needs small corrections to maintain speed — the 30A cap provides enough torque to hold against friction losses without the full-power spikes that duty-cycle mode would produce.

## State Machine Integration

The flywheel state machine manages transitions between phases:

```
IDLE → wantedState = RUNNING → SPINNING_UP (duty cycle bang-bang)
                                    ↓ (velocity enters tolerance, debounced)
                              AT_SETPOINT (torque current bang-bang)
                                    ↓ (velocity drops below tolerance)
                              SPINNING_UP (duty cycle bang-bang, recovery)
                                    ↓ (velocity recovers)
                              AT_SETPOINT ...
```

During rapid fire, the flywheel cycles between `AT_SETPOINT` and `SPINNING_UP` with each ball. The `AT_SETPOINT → SPINNING_UP` transition is also used to count launches.

## Debouncing

Two debouncers prevent instability:

### Control Mode Debouncer (`kFalling`, 40ms)

Prevents rapid toggling between duty-cycle and torque-current control when velocity oscillates around the tolerance boundary. `kFalling` means:
- **Entry to `AT_SETPOINT`:** Instant — switches to hold mode the first cycle velocity enters tolerance
- **Exit from `AT_SETPOINT`:** Delayed 40ms — holds in torque-current mode for 40ms after velocity dips below tolerance

This is intentional: at setpoint, the flywheel should be sticky. Brief noise or measurement jitter shouldn't cause a full-power duty-cycle spike. But a real velocity drop (from a ball) will persist longer than 40ms and trigger the recovery.

### Setpoint Debouncer (`kFalling`, 200ms)

Controls the external `isReadyToShoot()` signal consumed by the `ShooterStateMachine`. `kFalling` means:
- **Entry:** Instant — `isReadyToShoot()` goes true on first in-tolerance sample
- **Exit:** Delayed 200ms — stays true for 200ms after velocity drops

This keeps the shooter state machine's `FIRING` state stable through velocity dips caused by ball feed-through. The `ShooterStateMachine` latches in `FIRING` independently, so `isReadyToShoot()` only matters for the initial `PREPARING_TO_FIRE → FIRING` gate.

## Phoenix 6 Configuration Details

### Why `kP = 999999` Works

Phoenix 6's velocity closed-loop computes output as `kP * error`. With `kP = 999999`:
- Error of -0.001 RPS → output of -999.999 → saturates at peak limit
- Error of +0.001 RPS → output of +999.999 → saturates at peak limit (but reverse peak is 0)

This effectively creates a relay: any error in either direction produces a saturated output clamped by the configured peak limits. The `PeakReverseDutyCycle = 0` and `PeakReverseTorqueCurrent = 0` settings ensure the flywheel never actively brakes — it either drives forward or coasts.

### Why Two Different Control Modes

| | Duty Cycle (`VelocityDutyCycle`) | Torque Current (`VelocityTorqueCurrentFOC`) |
|---|---|---|
| Output unit | % of supply voltage | Amps of torque current |
| Peak limit | `PeakForwardDutyCycle = 1.0` | `PeakForwardTorqueCurrent = 30A` |
| Current limiting | Only stator/supply limits (150A/80A) | 30A hard cap from torque config |
| Use case | Spin-up: max power, fastest recovery | Hold: consistent torque, bounded current |

The torque current mode is specifically valuable at setpoint because it provides **consistent holding torque regardless of battery voltage**. Duty cycle output varies with voltage — at 12V, 100% duty = 12V, but at 10V it's only 10V. Torque current commands a specific current, so the motor applies the same force regardless of battery state.

## Comparison to Traditional PID

| Aspect | PID (`VelocityVoltage` Slot 2) | Bang-Bang (Two-Phase) |
|---|---|---|
| Recovery time | Gradual ramp — proportional to error | Instant full power — always maximum |
| Steady-state accuracy | Very precise — PID converges to zero error | Oscillates within tolerance band |
| Current draw | Smooth, proportional | Spiky — full on or off |
| Tuning complexity | 4+ gains (kP, kI, kD, kS, kV) | Tolerance window + debounce times |
| Best for | Precision single shots, consistent spin | Rapid fire, fastest recovery |
| Overshoot | Controllable via kD | Inherent — but bounded by torque limit at setpoint |

## Key Tuning Parameters

| Parameter | Default | Purpose |
|---|---|---|
| `TOLERANCE_RPM` | 100 RPM | Bandwidth for considering the flywheel "at setpoint" |
| `CONTROL_MODE_DEBOUNCE_SECONDS` | 0.04s | Prevents rapid duty-cycle/torque-current toggling |
| `AT_GOAL_DEBOUNCE_SECONDS` | 0.2s | Stability requirement before declaring ready to shoot |
| `PeakForwardTorqueCurrent` | 30A | Torque limit during hold phase — higher = more aggressive hold, more current |
| `StatorCurrentLimit` | 150A | Overall stator current safety limit |
| `SupplyCurrentLimit` | 80A | Battery-side current safety limit |

## References

- [Team 6328 (Mechanical Advantage) 2026 flywheel implementation](https://github.com/Mechanical-Advantage/RobotCode2026Public) — inspiration for the two-phase bang-bang approach
- [CTRE Phoenix 6 Documentation — Velocity Control](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html)
- [WPILib Debouncer](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html)
