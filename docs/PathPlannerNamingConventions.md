# PathPlanner Naming Conventions

**Version:** 1.0  
**Game:** REBUILT (2026)  
**Team:** FRC 360

This document defines standardized naming conventions for PathPlanner autonomous routines and paths based on the 2026 REBUILT game field layout and strategic objectives.

---

## Game Overview

**REBUILT™ presented by Haas** - Alliances score **fuel** into their **hub**, cross obstacles, and climb the **tower**. During autonomous (20 seconds), robots score fuel without driver input. Fuel can be pre-loaded, obtained from human players at the **depot**, or collected from the center field. During teleop (2:20), hubs alternate between active/inactive states. Robots climb the tower for bonus points as time runs out.

---

## Field Geography Reference

Based on the 2026 REBUILT game manual and `FieldConstants.java`, the field contains:

### Scoring Elements
- **Hub** (1 per alliance) - Primary scoring structure for fuel; alternates between active/inactive states
- **Tower** (1 per alliance) - Climbing structure with low/mid/high rungs for end-game points
- **Outpost** (1 per alliance) - Alliance-specific structure along the alliance wall

### Field Zones & Obstacles
- **Depot** (2 total) - 42" wide × 27" deep loading zone along alliance walls where human players provide fuel
- **Bumps** (4 total) - Terrain obstacles; two per alliance side (Left Bump, Right Bump)
- **Trenches** (4 total) - Protected obstacle areas; two per alliance side (Left Trench, Right Trench)
- **Neutral Zone** - Center field area where fuel can be collected

### Game Pieces
- **Fuel** - Game pieces that are scored into hubs

### Starting Positions (from driver station perspective)
- **Left** - Starting position on the left side
- **Middle** - Starting position in the center  
- **Right** - Starting position on the right side
- **Bump** - Starting position near a bump obstacle
- **Trench** - Starting position near a trench obstacle
- **Depot** - Starting position near the depot

---

## Auto Naming Convention

### Format
```
[ALLIANCE]_[SIDE]_[STARTING_POSITION]_[ACTION]
```

### Components

#### 1. ALLIANCE (Required)
- `BLUE` - Blue alliance auto
- `RED` - Red alliance auto

**Note:** Create mirror autos for both alliances when possible.

#### 2. SIDE (Required)
Where on the field the auto primarily operates:
- `DEPOT` - On depot side
- `OUTPOST` - On outpost side

#### 3. STARTING_POSITION (Required)
Starting configuration/location:
- `MIDDLE` - Center starting position
- `BUMP` - Near bump obstacle
- `TRENCH` - Near trench obstacle
- `DEPOT` - Near depot
- `OUTPOST` - Near outpost

#### 4. ACTION (Required)
Main strategy:

**Note:** All autos score and collect fuel - these are implicit.

- `MIDDLE_SPRINT` - Fast neutral zone fuel collection and scoring
- `BULLETPROOF` - Maximum reliability, minimal risk
- `AGGRESSIVE` - High-risk, high-reward
- `CLIMB` - Includes climbing
- `DEPOT_MIDDLE_CYCLE` - Cycle from depot to neutral zone fuel


### Examples

```
BLUE_OUTPOST_TRENCH_MIDDLE_SPRINT
RED_DEPOT_BUMP_BULLETPROOF
BLUE_OUTPOST_TRENCH_AGGRESSIVE
RED_DEPOT_TRENCH_SAFE
```


## Path Naming Convention

Paths are reusable trajectory segments that can be used across multiple autonomous routines. They should be named to clearly describe where they start, what happens during the path, and where they end.

### Format
```
[ALLIANCE]_[ORIGIN]_[ACTION]_[DESTINATION]
```

**All components except ORIGIN and DESTINATION are optional.** Include only what's necessary to make the path name clear and unambiguous.

### Components

#### 1. ALLIANCE (Optional)
Include **only when path geometry is alliance-specific** (different trajectories for red/blue due to field position or strategy). Omit if the path works for both alliances.
- `RED_` - Red alliance specific geometry
- `BLUE_` - Blue alliance specific geometry

#### 2. ORIGIN (Required)
Where the path starts from:
- `START_LEFT`, `START_CENTER`, `START_RIGHT` - Starting positions from driver station perspective
- `HUB` - Starting from hub area
- `DEPOT` - Starting from depot area
- `CENTERLINE` - Starting from centerline/neutral zone
- `BUMP`, `TRENCH` - Starting near/at obstacles
- `OUTPOST` - Starting from outpost area
- `TOP`, `BOTTOM`, `LEFT`, `RIGHT` - Directional qualifiers
- Can combine: `TOP_BUMP`, `BOTTOM_DEPOT`, `LEFT_TRENCH`, etc.

#### 3. ACTION (Optional)
What happens **during** the path. Omit if it's just a simple movement:
- `INTAKING` - Collecting fuel while driving
- `GO_OVER` - Going over an obstacle
- `GO_THROUGH` - Going through an area
- `GO_AROUND` - Avoiding an obstacle
- `POSITIONING` - Setting up for next action

#### 4. DESTINATION (Required)
Where the path ends:
- `HUB` - Ending at hub for scoring
- `CENTERLINE` - Ending at centerline/neutral zone
- `DEPOT` - Ending at depot
- `TOWER` - Ending at tower
- `BUMP`, `TRENCH`, `OUTPOST` - Ending at specific field elements
- `TOP`, `BOTTOM`, `LEFT`, `RIGHT` - Positional descriptors
- Can combine: `TOP_BUMP`, `BOTTOM_DEPOT`, `CENTERLINE_BOTTOM`, etc.

### Path Examples

#### Alliance-Agnostic Paths
Most paths should work for both alliances (omit alliance prefix):
```
START_CENTER_HUB               # Start center position to hub
CENTERLINE_HUB                 # Centerline to hub
DEPOT_CENTERLINE               # Depot to centerline
HUB_CENTERLINE                 # Hub to centerline
CENTERLINE_INTAKING_CENTERLINE_BOTTOM  # Collect fuel while moving within centerline
CENTERLINE_GO_OVER_TOP_BUMP    # Centerline, going over top bump
CENTERLINE_GO_THROUGH_BOTTOM_DEPOT    # Centerline through to bottom depot
HUB_GO_THROUGH_TRENCH          # Hub going through trench area
DEPOT_GO_AROUND_BUMP           # Depot avoiding bump obstacle
```

#### Alliance-Specific Paths
Only use alliance prefix when geometry differs by alliance:
```
RED_DEPOT_CENTERLINE_BOTTOM    # Red-specific depot to bottom centerline
BLUE_DEPOT_CENTERLINE_TOP      # Blue-specific depot to top centerline
RED_START_LEFT_HUB             # Red starting left to hub
BLUE_START_RIGHT_CENTERLINE    # Blue starting right to centerline
```

#### Test Paths
For development and testing:
```
TEST_FIGURE_8
TEST_STRAIGHT_LINE
TEST_ROTATION
TEST_SLALOM
```

### Examples

```
RED_MIDDLE_SPRINT_S1_TO_HUB
RED_MIDDLE_SPRINT_S2_TO_FUEL
RED_MIDDLE_SPRINT_S3_RETURN
RED_MIDDLE_SPRINT_S4_TO_FUEL
RED_MIDDLE_SPRINT_S5_TO_HUB

BLUE_BULLETPROOF_PRELOAD_TO_HUB
BLUE_BULLETPROOF_FUEL1_COLLECT
BLUE_BULLETPROOF_SCORE1_RETURN
BLUE_BULLETPROOF_SCORE2_TO_HUB

BLUE_LEFT_TRENCH_COLLECT_S1_THROUGH_TRENCH
BLUE_LEFT_TRENCH_COLLECT_S2_TO_FUEL
BLUE_LEFT_TRENCH_COLLECT_S3_TO_DEPOT
```

### Current Paths Renamed

| Current Name | Standardized Name | Reasoning |
|--------------|-------------------|-----------|
| `Red middle 1` | `START_CENTER_HUB` or `RED_START_CENTER_HUB` | Start center to hub (alliance-agnostic unless geometry differs) |
| `Red middle 2` | `HUB_INTAKING_CENTERLINE` | Hub to centerline while collecting fuel |
| `Red middle 3` | `CENTERLINE_HUB` | Return from centerline to hub |
| `Red middle 3.2` | `CENTERLINE_HUB_V2` | Alternate centerline to hub path |
| `Red middle 4` | `HUB_INTAKING_CENTERLINE_BOTTOM` | Hub to bottom centerline collecting fuel |
| `Red middle 4.2` | `HUB_INTAKING_CENTERLINE_BOTTOM_V2` | Alternate bottom centerline collection |
| `Blue 1` | `START_LEFT_HUB` or `BLUE_START_LEFT_HUB` | Starting left to hub |
| `Blue 2` | `HUB_INTAKING_CENTERLINE` | Hub to centerline collecting fuel |
| `Blue 3` | `CENTERLINE_HUB` | Centerline back to hub |
| `Example Path` | `TEST_EXAMPLE` | Test path |
| `INTAKING` | `TEST_INTAKING` | Test fuel collection |
| `SHOOTING` | `TEST_SHOOTING` | Test hub scoring |


---

## File Organization

### Auto Folder Structure (Recommended)
```
pathplanner/autos/
├── competition/
│   ├── BLUE_MIDDLE_MIDDLE_SPRINT.auto
│   ├── RED_MIDDLE_MIDDLE_SPRINT.auto
│   └── BLUE_DEPOT_LEFT_BULLETPROOF.auto
├── testing/
│   ├── TEST_SUPERSTRUCTURE.auto
│   └── BLUE_MIDDLE_TEST.auto
└── archive/
    └── old_deprecated_autos.auto
```

### Path Folder Structure (Recommended)
```
pathplanner/paths/
├── hub_scoring/
│   ├── START_CENTER_HUB.path
│   ├── START_LEFT_HUB.path
│   ├── START_RIGHT_HUB.path
│   ├── CENTERLINE_HUB.path
│   └── DEPOT_HUB.path
├── centerline_collection/
│   ├── HUB_INTAKING_CENTERLINE.path
│   ├── HUB_INTAKING_CENTERLINE_BOTTOM.path
│   ├── CENTERLINE_INTAKING_CENTERLINE_BOTTOM.path
│   └── DEPOT_INTAKING_CENTERLINE.path
├── obstacles/
│   ├── CENTERLINE_GO_OVER_TOP_BUMP.path
│   ├── HUB_GO_THROUGH_TRENCH.path
│   └── DEPOT_GO_AROUND_BUMP.path
├── depot_paths/
│   ├── DEPOT_CENTERLINE.path
│   ├── CENTERLINE_GO_THROUGH_BOTTOM_DEPOT.path
│   └── HUB_DEPOT.path
├── alliance_specific/
│   ├── RED_DEPOT_CENTERLINE_BOTTOM.path
│   ├── BLUE_DEPOT_CENTERLINE_TOP.path
│   ├── RED_START_LEFT_HUB.path
│   └── BLUE_START_RIGHT_HUB.path
└── test/
    ├── TEST_FIGURE_8.path
    └── TEST_STRAIGHT_LINE.path
```

---

## Best Practices

### DO
- ✅ Use UPPERCASE for all components
- ✅ Use underscores `_` to separate components
- ✅ Be descriptive and specific
- ✅ Omit alliance prefix unless geometry is alliance-specific
- ✅ Make paths reusable across multiple autos
- ✅ Version iteratively (`_V2`, `_V3`) when creating alternatives
- ✅ Group related paths in folders by function (hub_scoring, centerline_collection, etc.)
- ✅ Include ACTION component when something specific happens during the path

### DON'T
- ❌ Use spaces or special characters
- ❌ Use ambiguous abbreviations (e.g., `M1`, `P2`)
- ❌ Include alliance prefix when the path works for both alliances
- ❌ Tie paths to specific auto names (they should be reusable)
- ❌ Use lowercase or camelCase
- ❌ Create unnamed test files (`New Path`, `osjoj`, `jasjkas`)
- ❌ Use vague destinations (`TO_FUEL`, `TO_PIECE` - be specific: `CENTERLINE`, `DEPOT`)

---


## Quick Reference

### Auto Template
```
[ALLIANCE]_[SIDE]_[STARTING_POSITION]_[ACTION]

Examples:
BLUE_MIDDLE_MIDDLE_SPRINT
RED_DEPOT_BUMP_BULLETPROOF
BLUE_OUTPOST_TRENCH_AGGRESSIVE
```

### Path Template
```
[ALLIANCE]_[ORIGIN]_[ACTION]_[DESTINATION]

Examples:
START_CENTER_HUB
CENTERLINE_HUB
RED_DEPOT_CENTERLINE_BOTTOM
CENTERLINE_INTAKING_CENTERLINE_BOTTOM
CENTERLINE_GO_OVER_TOP_BUMP
HUB_GO_THROUGH_TRENCH
```

### Version Suffixes
```
_V2, _V3, _V4    - Revisions
```

---

## Notes

- This naming convention prioritizes **clarity**, **reusability**, and **searchability**
- All path names should be self-documenting (anyone should understand the path's purpose from its name)
- Paths are designed to be reusable across multiple autonomous routines
- Only include alliance prefix when path geometry is truly alliance-specific
- When in doubt, be more descriptive rather than less
- Update this document as new field navigation patterns emerge throughout the season

### Path Design Philosophy

- **Reusable by default** - Paths should work across multiple autos unless there's a specific reason they can't
- **Function over ownership** - Name describes what the path does and where it goes, not which auto uses it
- **Alliance-agnostic when possible** - Most paths should work for both red and blue alliances with field flipping
- **Composable** - Paths are building blocks that can be combined in different sequences to create various autonomous routines

### Common Patterns & Shorthand

- **All autos score and collect fuel** - These actions are implicit in every autonomous routine. Don't add `_COLLECT`, `_SCORE`, or `_HUB_SCORE` to names.
- **`MIDDLE_SPRINT`** - Team convention for collecting dozens of fuel from the neutral zone (center field) and scoring in the hub rapidly.
- **`BULLETPROOF`** - Conservative, high-reliability autonomous focused on guaranteed points with minimal risk.
- **`PRELOAD`** - Minimal auto that scores only the fuel pre-loaded into the robot at match start.
- **No defensive autos** - All autonomous routines are offensive (scoring-focused).

