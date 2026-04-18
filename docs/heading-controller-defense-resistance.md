# Heading Controller: Defense Resistance Design

## Goal

Make the robot highly resistant to opposing robots physically rotating us while we are shooting.
During defense, a robot can bump or push against our drivetrain to spin us off-target. We need the
heading controller to snap back to setpoint quickly, tolerate brief disturbances without missing
shots, and not allow integral windup to create a persistent offset after a hit.

---

## Current State

| Parameter | Value |
|-----------|-------|
| Kp | 15.0 |
| Ki | 0.2 |
| Kd | 1.0 |
| I-Zone | 10° |
| Static tolerance | 3.0° |
| Speed tolerance | 1.0° per m/s |

The heading controller runs in software (WPILib `PIDController`) on the RIO, outputting an omega
setpoint fed into `FieldCentricFacingAngle`. Izone is already set to 10°, meaning the integrator
only accumulates when we are within 10° of setpoint.

---

## Problem: Residual Integral Accumulation Under Defense

When a defender pushes us off target:

1. If the push is large (> 10°), the integrator freezes at its current value (Izone kicks out).
2. We snap back with Kp + Kd. So far so good.
3. But if the push is small (< 10°) and sustained, Ki accumulates a nonzero integral.
4. After the defender releases, we are now *at* setpoint but the integral still holds a residual
   value — effectively a constant torque bias. This creates a steady-state heading offset in the
   direction the defender was pushing.

---

## Option A: Leaky Integrator

Each periodic cycle, decay the integral by a small factor:

```java
integral = integral * LEAK_FACTOR + Ki * error * dt;
```

With `LEAK_FACTOR ≈ 0.95` (per 20 ms cycle), the integral's time constant is roughly:

```
τ = -dt / ln(LEAK_FACTOR) ≈ 0.39 s
```

**Pros:**
- Residual integral from a sustained push drains away in ~1 second after contact ends.
- Prevents indefinite windup regardless of Izone.
- Natural behavior: the integrator holds its value during sustained disturbance, releases after.

**Cons:**
- WPILib's `PIDController` does not natively support a leaky integrator. We would need to manage
  the integral ourselves and switch to a manual PID loop, or subclass / wrap `PIDController`.
- Slightly more code complexity.
- If the leak is too aggressive, Ki loses effectiveness entirely.

**Implementation sketch (manual PID):**

```java
double integral = 0.0;
static final double LEAK_FACTOR = 0.95;    // per 20 ms loop cycle
static final double HEADING_KI_LEAK = 0.2; // same Ki value

// In periodic / calculate():
double error = setpoint - measurement; // radians, continuous-wrapped
integral = integral * LEAK_FACTOR + HEADING_KI_LEAK * error * dt;
double output = HEADING_KP * error + integral + HEADING_KD * (error - prevError) / dt;
prevError = error;
```

---

## Option B: Feedforward Based on Drivetrain Force (Anti-Defense FF)

Instead of fixing the integrator, add a **disturbance feedforward** that detects when an external
torque is rotating us and immediately counteracts it.

**Concept:** Compare the commanded omega (from the heading controller) to the actual measured omega
(from the gyro). The difference is approximately the disturbance (defender pushing us):

```
disturbance_omega = measured_omega - commanded_omega
FF_output = disturbance_omega * K_FF
```

Feed this forward as additional omega to counteract the push.

**Pros:**
- Reacts immediately — no integral accumulation delay.
- Directly addresses the physical cause: an unexpected external rotation rate.
- Does not touch the integrator at all.
- Conceptually simple and fast.

**Cons:**
- `measured_omega - commanded_omega` also captures normal tracking lag, not just defense.
  Careful tuning of `K_FF` required to avoid amplifying normal control noise.
- Only works while `faceAngleWhileDriving` is active (we have commanded omega available).
- May interact poorly with aggressive Kd — both react to sudden omega changes.

**Implementation sketch:**

```java
// In faceAngleWhileDriving():
double commandedOmega = angleFacingRequest.HeadingController.getLastAppliedOutput();
double measuredOmega  = getVelocity().omegaRadiansPerSecond;
double disturbance    = measuredOmega - commandedOmega;
double defenseFF      = disturbance * K_DEFENSE_FF; // K_FF ∈ [0.1, 0.5], tune on field

this.setControl(
    angleFacingRequest
        .withVelocityX(fieldRelativeVelXMps)
        .withVelocityY(fieldRelativeVelYMps)
        .withTargetDirection(targetHeading)
        .withRotationalDeadband(/* apply defenseFF separately */));
```

> **Note:** `FieldCentricFacingAngle` does not have a direct "add omega" input on top of the
> heading controller output. To add the FF term cleanly we would need to either:
> - Bypass `FieldCentricFacingAngle` and compute omega = `headingController.calculate() + defenseFF`
>   then pass it directly into a `FieldCentric` request with explicit omega, **or**
> - Shift the heading setpoint slightly to pre-compensate (less clean).

---

## Recommendation

**Start with the leaky integrator (Option A).** The described symptom — residual integral
accumulation causing a constant heading offset after contact ends — is exactly what a leaky
integrator fixes. It is also lower risk than feedforward, which requires on-field calibration of
`K_FF` and can cause oscillation if set too high.

If the leaky integrator is not enough (robot is still being rotated significantly during a shot),
**layer in feedforward (Option B)** on top. The two approaches are complementary:

- Leaky Ki handles slow, sustained pushes where integral winds up.
- Defense FF handles sharp, fast impacts where Kp + Kd alone cannot react fast enough.

### Suggested Leaky Integrator Tuning Steps

1. Set `LEAK_FACTOR = 0.97` (conservative — ~0.65 s time constant). Observe if residual offset
   disappears after simulated defense contact.
2. If integral drains too fast and steady-state error reappears during static hold, lower leak to
   `0.99` (~1.9 s time constant).
3. If residual offset still persists, lower to `0.95` (~0.39 s).
4. After choosing leak factor, re-tune Ki if needed — leaky Ki requires slightly higher Ki to
   achieve the same steady-state correction as a standard integrator.

---

## Related Code

| File | Key lines |
|------|-----------|
| [CommandSwerveDrivetrain.java](../src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) | Heading PID gains ~L94, `faceAngleWhileDriving` ~L279, `isAlignedToTarget` ~L612 |
| [XOutWhileAligningCommand.java](../src/main/java/frc/robot/commands/XOutWhileAligningCommand.java) | X-out state machine, 1.5° tolerance |
| [ShooterStateMachine.java](../src/main/java/frc/robot/subsystems/Shooter/ShooterStateMachine.java) | `isAlignedToTarget` gate for firing |
