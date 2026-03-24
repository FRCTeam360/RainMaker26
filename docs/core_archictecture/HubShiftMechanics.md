# Hub Shift Mechanics

This document describes how RainMaker26 tracks which alliance hub is active during teleop, how the
display values (`Can Score in Hub`, `Time Left in Phase`) are computed, and why the TOF + sensor
delay offset is applied.

---

## Table of Contents

1. [Game Rule Summary](#game-rule-summary)
2. [Phase Timeline](#phase-timeline)
3. [Grace Periods](#grace-periods)
4. [Time-of-Flight Offset](#time-of-flight-offset)
5. [Display vs. Shooter Gate](#display-vs-shooter-gate)
6. [Implementation Reference](#implementation-reference)

---

## Game Rule Summary

During the 2026 teleop period, only **one alliance's hub** accepts balls at a time. The active hub
alternates on a fixed schedule (the "shifts"). During autonomous and endgame, **both hubs** are
always active.

The alliance that **won autonomous** (`autoWinner`) determines which hub is active during each
shift:

| Phase            | Active Hub  |
| ---------------- | ----------- |
| Autonomous       | BOTH        |
| Transition       | BOTH        |
| Shift 1 (teleop) | AUTOLOSER   |
| Shift 2 (teleop) | AUTOWINNER  |
| Shift 3 (teleop) | AUTOLOSER   |
| Shift 4 (teleop) | AUTOWINNER  |
| Endgame          | BOTH        |

`AUTOLOSER` means only the alliance that **lost** auto can score; `AUTOWINNER` means only the
alliance that **won** auto can score.

---

## Phase Timeline

All times are seconds remaining on the match clock (`DriverStation.getMatchTime()`).

```
Raw match time (seconds remaining)
────────────────────────────────────────────────────────────────────
150        130        105        80         55         30         0
 │          │          │          │          │          │          │
 │Prematch  │Transition│ Shift 1  │ Shift 2  │ Shift 3  │ Shift 4  │Endgame│
 │          │  BOTH    │AUTOLOSER │AUTOWINNER│AUTOLOSER │AUTOWINNER│ BOTH  │
```

Raw boundary constants (in `RobotUtils`):

| Constant                  | Value (s) | Meaning                          |
| ------------------------- | --------- | -------------------------------- |
| `TRANSITION_END_SECONDS`  | 130       | Transition ends, shift 1 begins  |
| `SHIFT_1_END_SECONDS`     | 105       | Shift 1 ends, shift 2 begins     |
| `SHIFT_2_END_SECONDS`     | 80        | Shift 2 ends, shift 3 begins     |
| `SHIFT_3_END_SECONDS`     | 55        | Shift 3 ends, shift 4 begins     |
| `ENDGAME_START_SECONDS`   | 30        | Shift 4 ends, endgame begins     |

---

## Grace Periods

At each shift boundary, the hub that was **active goes inactive immediately**. The hub that was
**inactive gets a 2-second grace window** (`SHIFT_GRACE_PERIOD_SECONDS = 2.0 s`) where **both**
hubs are simultaneously active — this allows balls that were already in flight at the boundary
moment to still score.

### What this means per alliance

| Event                     | Our hub    | Opponent's hub |
| ------------------------- | ---------- | -------------- |
| Our hub goes **active**   | Active immediately | Gets 2 s grace (still active) |
| Our hub goes **inactive** | Inactive immediately | Active immediately |

**Key insight:** The grace period is always for the *other* team's in-flight balls. From our
perspective, we go active and inactive at the exact raw boundary — no grace delay in either
direction.

### Shooter gate (`getActiveHub`)

The shooter gate uses the full grace-period logic (with `BOTH` during the 2-second window) so that
any ball we fired just before the boundary will still be allowed to count. This is the safe,
conservative behavior for the hardware gate.

### Display (`getDisplayActiveHub`)

The display folds grace into the active window silently. Since our hub is already active during
the grace window, the driver sees a clean binary flip at each raw boundary with no 2-second
intermediate state. This matches driver intuition: "my hub just went active / inactive."

---

## Time-of-Flight Offset

The driver needs to **stop shooting slightly before** the raw hub-inactive boundary, because a
ball fired at the boundary moment will still be in the air when the hub closes. Similarly, the
driver can **start shooting slightly before** the raw hub-active boundary, because the ball will
land while the hub is already open.

The offset applied is:

```
effectiveTOF = max(currentTOF, minMappedTOF) + BALL_TO_SENSOR_DELAY_SECONDS
```

| Component                       | Value    | Meaning                                                    |
| ------------------------------- | -------- | ---------------------------------------------------------- |
| `currentTOF`                    | varies   | Calculated time-of-flight for current robot position       |
| `minMappedTOF`                  | varies   | Floor from the shot map (handles out-of-range poses)       |
| `BALL_TO_SENSOR_DELAY_SECONDS`  | 0.5 s    | Extra travel time after entering hub before scoring sensor |

The **adjusted match time** used for all display and shooter-gate decisions is:

```
matchTimeAdjusted = matchTimeRaw - effectiveTOF
```

Using `matchTimeAdjusted` instead of `matchTimeRaw` means:
- When `matchTimeAdjusted` crosses a boundary, a ball fired **right now** will land exactly at
  that boundary.
- `Can Score in Hub` turns **on** slightly before the raw boundary → driver can fire immediately.
- `Can Score in Hub` turns **off** slightly before the raw boundary → driver stops firing early
  enough that the last ball lands before the hub closes.
- `Time Left in Phase` hits **zero** at the same moment `Can Score in Hub` flips.

---

## Display vs. Shooter Gate

Two separate hub-active computations run every cycle:

### `cachedIsHubActive` — shooter gate

- Computed via `getActiveHub(matchTimeRaw, isTeleop, effectiveTOF)` → `isHubActiveForAlliance()`
- Uses the full grace-period logic (BOTH during 2 s window)
- Controls `canShootToTarget()` in the superstructure — prevents firing when hub is inactive

### `cachedDisplayIsHubActive` — Elastic dashboard

- Computed via `getDisplayActiveHub(matchTimeAdjusted)` → `isHubActiveForAlliance()`
- Grace folded into active; clean binary flip at raw boundaries on TOF-adjusted time
- Published as `SmartDashboard / "Can Score in Hub"`
- Flips at the **exact same moment** as `primaryTimeLeft` resets to zero

### `primaryTimeLeft` — Elastic countdown

- Computed via `getDisplayTimeUntilHubChange(matchTimeAdjusted)`
- Counts down to the next raw boundary on TOF-adjusted time
- During AUTO and ENDGAME, shows raw `matchTimeRaw` instead (no shift boundaries to count to)
- Uses identical boundary constants as `getDisplayActiveHub` so both always change simultaneously

---

## Implementation Reference

| Symbol | File | Purpose |
| ------ | ---- | ------- |
| `getActiveHub()` | `RobotUtils.java` | Shooter gate: which hubs are accepting, with grace |
| `getDisplayActiveHub()` | `RobotUtils.java` | Display: which hub is active, grace folded in |
| `getDisplayTimeUntilHubChange()` | `RobotUtils.java` | Display countdown to next phase flip |
| `isHubActiveForAlliance()` | `RobotUtils.java` | Maps BOTH/AUTOWINNER/AUTOLOSER → boolean for our alliance |
| `getAutoWinner()` | `RobotUtils.java` | Parses game-specific message to determine auto winner |
| `cachedIsHubActive` | `SuperStructure.java` | Shooter gate result, updated every cycle |
| `cachedDisplayIsHubActive` | `SuperStructure.java` | Display result, published to SmartDashboard |
| `primaryTimeLeft` | `SuperStructure.java` | Countdown value, published to SmartDashboard |
| `getHubShiftTimeOfFlight()` | `SuperStructure.java` | Computes effectiveTOF = max(TOF, minTOF) + sensorDelay |
| `BALL_TO_SENSOR_DELAY_SECONDS` | `SuperStructure.java` | 0.5 s extra travel after hub entry before scoring sensor |
| `SHIFT_GRACE_PERIOD_SECONDS` | `RobotUtils.java` | 2.0 s grace window at each shift boundary |
