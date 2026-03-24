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

| Event                       | Our hub                                     | Opponent's hub                        |
| --------------------------- | ------------------------------------------- | ------------------------------------- |
| Our hub goes **active**     | Active immediately at raw boundary          | Gets 2 s grace (also still active)   |
| Our hub goes **inactive**   | Inactive immediately at raw boundary        | Active immediately; we get 2 s grace  |

**Key insight:** When our hub goes **inactive**, we get the 2-second grace window — any ball we
fired before the raw boundary will land while both hubs are still accepting. This means the driver
does **not** need to stop shooting early; the grace window covers all in-flight balls.

When our hub goes **active**, there is no delay for us — we are active at the raw boundary
immediately. The opponent gets the grace window (they were just active), but that does not affect
when we can start scoring.

### Shooter gate (`getActiveHub`)

Uses the full grace-period logic (returns `BOTH` during the 2-second window). This correctly
allows balls fired just before our hub went inactive to still register as scored.

### Display (`getDisplayActiveHub`)

Grace is folded silently into the adjacent active window. The driver never sees a separate
"grace" phase — the hub either is or isn't active for them. See
[Time-of-Flight Offset](#time-of-flight-offset) for how the boundary timing differs between
going active and going inactive.

---

## Time-of-Flight Offset

The driver needs to **start shooting slightly before** the raw hub-active boundary, because a
ball fired now will land when the hub has just opened. However, the driver can shoot right up
**until the raw hub-inactive boundary** — any ball already in flight when the hub closes will
still score during the 2-second grace period, which is always longer than our TOF + sensor delay.

The offset applied is:

```
effectiveTOF = max(currentTOF, minMappedTOF) + BALL_TO_SENSOR_DELAY_SECONDS
```

| Component                       | Value    | Meaning                                                    |
| ------------------------------- | -------- | ---------------------------------------------------------- |
| `currentTOF`                    | varies   | Calculated time-of-flight for current robot position       |
| `minMappedTOF`                  | varies   | Floor from the shot map (handles out-of-range poses)       |
| `BALL_TO_SENSOR_DELAY_SECONDS`  | 0.5 s    | Extra travel time after entering hub before scoring sensor |

The **adjusted match time** is:

```
matchTimeAdjusted = matchTimeRaw - effectiveTOF
```

### Asymmetric offset

The TOF offset is applied **asymmetrically**:

| Transition            | Time used          | Reason                                                                 |
| --------------------- | ------------------ | ---------------------------------------------------------------------- |
| Hub goes **active**   | `matchTimeAdjusted` | Shoot early — ball lands exactly when hub opens                       |
| Hub goes **inactive** | `matchTimeRaw`      | Grace period (2 s) covers in-flight balls; no early cutoff needed     |

`Can Score in Hub` is therefore: active if **either** `matchTimeAdjusted` or `matchTimeRaw`
evaluates to our hub being active. This means:
- Turns **ON** `effectiveTOF` seconds before the raw active boundary (adjusted crosses first)
- Turns **OFF** exactly at the raw inactive boundary (raw crosses; adjusted already crossed but
  OR with raw keeps it true until raw also crosses)

`Time Left in Phase` uses `matchTimeAdjusted` throughout so it hits zero at the moment
`Can Score in Hub` turns ON (the driver-relevant event at the start of our active window).

---

## Display vs. Shooter Gate

Two separate hub-active computations run every cycle:

### `cachedIsHubActive` — shooter gate

- Computed via `getActiveHub(matchTimeRaw, isTeleop, effectiveTOF)` → `isHubActiveForAlliance()`
- Uses the full grace-period logic (BOTH during 2 s window)
- Controls `canShootToTarget()` in the superstructure — prevents firing when hub is inactive

### `cachedDisplayIsHubActive` — Elastic dashboard

- Computed as the OR of two `isHubActiveForAlliance()` calls:
  - `getDisplayActiveHub(matchTimeAdjusted)` — turns ON early by TOF
  - `getDisplayActiveHub(matchTimeRaw)` — keeps ON until the raw boundary
- Grace folded into active; no intermediate grace phase visible to driver
- Published as `SmartDashboard / "Can Score in Hub"`
- Turns ON `effectiveTOF` seconds before the raw active boundary; turns OFF exactly at the raw
  inactive boundary

### `primaryTimeLeft` — Elastic countdown

- During **AUTO**: shows raw `matchTimeRaw` (countdown to end of auto)
- During **ENDGAME** (raw time has also crossed `ENDGAME_START_SECONDS`): shows raw `matchTimeRaw`
  (countdown to end of match)
- During **TRANSITION**: shows `matchTimeRaw - TRANSITION_END_SECONDS` — raw seconds until shift 1
  begins. No TOF offset because both hubs are active and there is no inactive boundary imminent.
- During **teleop shifts** (including the TOF-early window entering endgame where `matchTimeAdjusted`
  has crossed `ENDGAME_START_SECONDS` but `matchTimeRaw` has not): the time base is chosen based on
  whether our hub is **currently active**:
  - **Currently active** → use `matchTimeRaw`. Counting toward going inactive (or staying active
    into endgame). Grace covers in-flight balls so no early cutoff needed; countdown matches clock.
  - **Currently inactive** → use `matchTimeAdjusted`. Counting toward going active (including
    endgame BOTH). Countdown hits zero exactly when a ball fired now would land while the hub is open.
- In both teleop cases the countdown value is `getDisplayTimeUntilHubChange(timeForCountdown)`
- `Can Score in Hub` turning **ON** coincides with the countdown resetting (adjusted time crosses
  the active boundary); `Can Score in Hub` turning **OFF** happens `effectiveTOF` seconds later
  at the raw boundary, after the countdown has already reset for the next phase

---

## Implementation Reference

| Symbol | File | Purpose |
| ------ | ---- | ------- |
| `getActiveHub()` | `RobotUtils.java` | Shooter gate: which hubs are accepting, with grace |
| `getDisplayActiveHub()` | `RobotUtils.java` | Display: which hub is active, grace folded in |
| `getDisplayTimeUntilHubChange()` | `RobotUtils.java` | Display countdown to next phase flip |
| `isHubActiveForAlliance()` | `RobotUtils.java` | Maps BOTH/AUTOWINNER/AUTOLOSER → boolean for our alliance |
| `getAutoWinner()` | `RobotUtils.java` | Parses game-specific message to determine auto winner |
| `HubShiftTracker` | `utils/HubShiftTracker.java` | Stateful tracker — owns all per-cycle hub shift computation |
| `HubShiftTracker.update()` | `HubShiftTracker.java` | Must be called once per cycle; reads DriverStation, updates all cached state |
| `HubShiftTracker.isHubActive()` | `HubShiftTracker.java` | Shooter gate result (full grace logic) |
| `HubShiftTracker.isHubActiveForDisplay()` | `HubShiftTracker.java` | Display "Can Score in Hub" (asymmetric TOF logic) |
| `HubShiftTracker.getPrimaryTimeLeft()` | `HubShiftTracker.java` | "Time Left in Phase" countdown value |
| `HubShiftTracker.getTeleopShift()` | `HubShiftTracker.java` | Current shift number (1–4), 0 outside teleop |
| `HubShiftTracker.getActiveHub()` | `HubShiftTracker.java` | Raw `ActiveHub` enum used by the shooter gate this cycle |
| `HubShiftTracker.log()` | `HubShiftTracker.java` | Publishes all values to SmartDashboard and AdvantageKit Logger |
| `hubShiftTracker` | `SuperStructure.java` | Instance owned by `SuperStructure`, updated in `periodic()` |
| `canScoreAtHub()` | `SuperStructure.java` | Public accessor delegating to `hubShiftTracker.isHubActive()` |
| `BALL_TO_SENSOR_DELAY_SECONDS` | `HubShiftTracker.java` | 1.0 s extra travel after hub entry before scoring sensor |
| `SHIFT_GRACE_PERIOD_SECONDS` | `RobotUtils.java` | 2.0 s grace window at each shift boundary |
