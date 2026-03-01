# Shoot-on-the-Move: Findings & Optimization Roadmap

## Current State

Our `ShotCalculator` is adapted from FRC 6328 Mechanical Advantage and has the right architecture:
- Velocity decomposition into field frame
- Shooter offset compensation (7" behind robot center, includes omega contribution)
- Time-of-flight lookahead concept
- Interpolation maps for hood angle, flywheel speed, and TOF
- Per-loop caching via `preSchedulerUpdate()`

**However, the velocity compensation is currently a no-op.** The TOF map has a single entry `(0.0, 0.0)`, so `timeOfFlightSecs` always returns 0.0, the lookahead offset is always zero, and the calculator behaves as a stationary shot calculator.

Static aiming (distance-to-hood-angle, distance-to-flywheel-speed, heading computation) works correctly for standing shots.

---

## Comparison with Top Teams

### FRC 6328 Mechanical Advantage (2026)

Their `LaunchCalculator` builds on the same core concept but adds:

| Feature | Us | 6328 |
|---|---|---|
| TOF convergence | Single-pass | 20-iteration convergence loop |
| Phase delay compensation | None | 30ms twist extrapolation |
| Heading control | CTRE `FieldCentricFacingAngle` PID | PD + feedforward (d(heading)/dt via moving average filter) |
| Hood tracking | Position setpoint only | Position + velocity feedforward (d(angle)/dt) |
| Velocity limiting | None | Polar velocity limit using law of sines (0.6 rad/s max at target) |
| Center of rotation | Robot center always | Interpolated: robot center (error > 30 deg) to launcher (error < 15 deg) |
| Launcher lateral offset | 180 deg rotation | arcsine correction for exact lateral angular error |
| Velocity source | Measured (`drivetrain.getVelocity()`) | Commanded setpoints (less noisy) |

Key insight: their 20-iteration loop resolves the circular dependency where TOF depends on distance, but distance depends on the virtual position, which depends on TOF. At low speeds the single-pass is close enough.

### FRC 111 WildStang (2026)

Their 2026 code isn't public yet. From their Chief Delphi build blog:
- Work in the field frame where the hub is stationary
- Robot velocity is an additive term on the projectile's initial velocity vector
- Avoids needing iterative TOF solve (per mentor roni.harnik)
- Uses empirical TOF-vs-distance measurements rather than physics simulation
- Their 2022 implementation decomposed velocity into parallel/perpendicular components relative to the target line and applied empirical scale factors

---

## Optimization Roadmap

### Phase 1: Get It Working (First Comp)

**Goal:** Functional shoot-on-the-move at limited driving speeds (1-2 m/s).

#### 1. Populate the TOF map with real data

Measure ball flight times at several distances using slow-motion video (240fps phone cameras work). Fire from a standstill at known distances and time from ball-leaves-shooter to ball-hits-target.

```java
// Example values -- replace with actual measurements
timeOfFlightMap.put(1.0, 0.4);
timeOfFlightMap.put(1.5, 0.5);
timeOfFlightMap.put(2.0, 0.6);
timeOfFlightMap.put(2.5, 0.7);
timeOfFlightMap.put(3.0, 0.8);
timeOfFlightMap.put(3.5, 0.9);
timeOfFlightMap.put(4.0, 1.0);
timeOfFlightMap.put(5.0, 1.15);
```

More data points = smoother interpolation. Prioritize the 1.5-4m range where most shots happen.

#### 2. Add a velocity cap to the shooting command

In the `faceAngleWhileDrivingCommand` binding (or a wrapper), scale down driver input while the shoot trigger is held. This limits how much compensation the calculator needs to do, keeping errors small while the single-pass lookahead is "good enough."

Start with 40-50% max speed and tune up from there.

#### 3. Validate with AdvantageKit logging

The calculator already logs `lookaheadPose`, `distanceToTarget`, `timeOfFlightSecs`, etc. Use these to verify:
- `timeOfFlightSecs` is non-zero and reasonable
- `lookaheadPose` visibly offsets from robot pose when moving
- `targetHeading` leads the target when strafing

Drive in a circle around the hub and watch the logs. The lookahead pose should be "ahead" of the robot in the direction of travel.

---

### Phase 2: Improve Accuracy (Between Comps)

**Goal:** Reliable shots at moderate speeds (2-3 m/s) from typical match distances.

#### 4. Add iterative TOF convergence

Replace the single-pass lookahead with a convergence loop. Small code change in `ShotCalculator.calculateShot()`:

```java
double timeOfFlightSecs = timeOfFlightMap.get(shooterToTargetDistanceMeters);
Translation2d lookaheadPosition = shooterPosition.getTranslation();
double lookaheadDistanceMeters = shooterToTargetDistanceMeters;

for (int i = 0; i < 10; i++) {
    timeOfFlightSecs = timeOfFlightMap.get(lookaheadDistanceMeters);
    lookaheadPosition =
        shooterPosition.getTranslation()
            .plus(new Translation2d(
                shooterVelXMps * timeOfFlightSecs,
                shooterVelYMps * timeOfFlightSecs));
    lookaheadDistanceMeters = target.getDistance(lookaheadPosition);
}
```

This converges in 3-5 iterations typically; 10 is safe overhead. Zero allocation cost since it's just doubles.

#### 5. Add phase delay compensation

Account for the ~20-30ms pipeline latency (sensor read to motor output) by extrapolating the robot pose forward before calculating:

```java
Pose2d robotPosition = robotPoseSupplier.get();
ChassisSpeeds robotSpeeds = velocitySupplier.get();
double phaseDelaySecs = 0.025; // tune this

robotPosition = robotPosition.exp(new Twist2d(
    robotSpeeds.vxMetersPerSecond * phaseDelaySecs,
    robotSpeeds.vyMetersPerSecond * phaseDelaySecs,
    robotSpeeds.omegaRadiansPerSecond * phaseDelaySecs));
```

This goes at the top of `calculateShot()` before any other computation.

#### 6. Refine the interpolation maps

With more practice time, add data points to tighten the hood angle and flywheel speed maps. Focus on the distances where you see the most variance. Mark which values are validated on the real robot (like the existing `// THIS IS GOOD` comments).

---

### Phase 3: Push the Envelope (Later Comps / Offseason)

**Goal:** Accurate shots at high speeds (3+ m/s) and from wider angles.

#### 7. Heading feedforward

Instead of relying solely on CTRE's `FieldCentricFacingAngle` PID to chase the moving heading setpoint, compute the rate of change of the heading and feed it forward:

- Track `lastHeading` and compute `d(heading)/dt` each loop using a moving average filter
- Add this as a feedforward omega term to the drive request
- This makes heading tracking proactive rather than reactive, which matters at speed

#### 8. Polar velocity limiting

Prevent the driver from strafing so fast that the shot compensation breaks down. Use the law of sines to compute the max linear velocity based on the angle of travel relative to the target:

- Movement toward/away from the hub: no limit needed
- Pure tangential strafing: most constrained
- The limit scales with distance (farther = more forgiving angular error at the target)

Cap the max angular rate of the ball at the target to ~0.5-0.6 rad/s and work backwards to a linear speed limit.

#### 9. Center-of-rotation shifting

When heading error is large, rotate around the robot center for fast corrections. As error shrinks below ~15 degrees, smoothly shift the center of rotation to the shooter position for more precise fine-tuning. This reduces the coupling between translation and rotation for the shooter.

#### 10. Hood velocity feedforward

Compute `d(hoodAngle)/dt` via moving average filter and feed it forward to the hood motor. This keeps the hood actively tracking the changing angle rather than always lagging behind the position setpoint.

#### 11. Use commanded velocity instead of measured

Consider switching from `drivetrain.getVelocity()` (measured, noisy) to the commanded velocity setpoints (what we asked the drivetrain to do). The commanded signal is smoother and avoids injecting sensor noise into the shot calculation. The tradeoff is that it assumes the drivetrain is tracking well, which it should be at steady-state.

---

## Testing Checklist

For each phase, validate with these tests:

- [ ] **Standing shots at known distances** -- confirm no regression from stationary accuracy
- [ ] **Slow strafe shots (1 m/s)** -- ball should land close to stationary accuracy
- [ ] **Medium strafe shots (2 m/s)** -- ball should consistently hit the target
- [ ] **Drive-toward/away shots** -- verify distance compensation is correct direction
- [ ] **Circular orbit shots** -- drive in a circle around the hub and fire continuously
- [ ] **Log review** -- verify lookahead pose, TOF, and heading values are sane in AdvantageKit

## References

- [6328 RobotCode2026Public](https://github.com/Mechanical-Advantage/RobotCode2026Public) -- `LaunchCalculator.java`, `DriveCommands.java`
- [111 2026 Build Blog](https://www.chiefdelphi.com/t/wildstang-robotics-program-team-111-and-112-build-blog-2026/509853) -- Posts #117, #123, #156
- [Shoot on the Move from the Code Perspective (Chief Delphi)](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815)
