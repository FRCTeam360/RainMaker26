# Plan: BLine Architecture Document

## Context

RainMaker26 uses both PathPlanner and BLine for autonomous path following. PathPlanner is used for absolute performance (time-parameterized, GUI-designed); BLine is used for consistency and real-world robustness (polyline-based, collision-aware).

PathPlanner already has a clean "define once, generate all 4 sides" workflow via Python scripts. BLine currently duplicates this manually — `BLineAutos.java` has 4 copies of every auto method (MASTER, MIRRORED, FLIPPED, FLIPPED MIRRORED).

The goal is a `BLineAutoFactory` pattern that mirrors PathPlanner's approach: define one MASTER Red Right auto in code, register all 4 variants automatically — while keeping all **paths as GUI-editable files** (not Java code).

## Deliverable

Create **`docs/BLineArchitecture.md`** — an architecture reference document.

---

## Document Outline

### 1. Overview
- Dual-system: PathPlanner (performance) + BLine (consistency/collision avoidance)
- Both feed the same `SendableChooser<Command>` in `RobotContainer`
- Both use a "1 MASTER → 4 variants" pattern, implemented differently
- **Paths in both systems are files, editable via their respective GUIs**

### 2. PathPlanner Architecture (Reference)
- MASTER Red Right `.auto` and `.path` files designed in PathPlanner GUI
- Python scripts (`flip_autos.py`, `mirror_autos.py`, `generate_all_autos.py`) generate the 3 other variants offline
- `AutoBuilder.buildAutoChooser()` auto-discovers all `.auto` files at startup
- Transformation math (done by Python, stored in output files):
  - Mirror: `new_y = FIELD_WIDTH - old_y`, `rotation = -rotation`
  - Flip: `new_x = FIELD_LENGTH - old_x`, `rotation += 180°`

### 3. BLine Architecture (Proposed)

#### Path Files
- All BLine paths are created and edited in the **BLine GUI**, stored as path files on disk
- Loaded in Java by name: `new Path("name")` — identical to how `new Path("drive test")` already works
- MASTER paths are defined as Red Right geometry in the GUI
- No `BLinePaths.java` — path geometry lives in files, not code
- Naming convention mirrors PathPlanner: descriptive, reusable, alliance-agnostic where possible

#### Core: `BLineAutoFactory`
A factory class with a `Transform` context that:
1. Encapsulates which of the 4 transformations is active (NONE / MIRROR / FLIP / FLIP_MIRROR)
2. Wraps path loading: `factory.path("name")` loads by name then applies the active transform
3. Exposes auto composition helpers (`pathWithIntake`, `followPath`, etc.) that use `factory.path()`
4. Exposes a static `registerAll()` that creates all 4 factory instances and registers the auto 4 times

```
Transform enum:
  NONE           → "MASTER Red Right"               (path loaded as-is)
  MIRROR         → "MIRRORED Red Left"              (path.mirrorY())
  FLIP           → "FLIPPED Blue Right"             (path.flip())
  FLIP_MIRROR    → "FLIPPED MIRRORED Blue Left"     (path.flip() + mirrorY())
```

#### Path Loading & Transformation
```
factory.path("START_CENTER_HUB")
  NONE        → new Path("START_CENTER_HUB")                    // as-is
  MIRROR      → new Path("START_CENTER_HUB").mirror()           // Y-axis mirror
  FLIP        → new Path("START_CENTER_HUB").flip()             // BLine built-in
  FLIP_MIRROR → new Path("START_CENTER_HUB").flip().mirror()    // both
```
- Uses BLine's built-in `path.flip()` for the X-axis flip (Red→Blue)
- Y-axis mirror applied via field-width arithmetic on waypoints (stays in factory, not in path files)

#### Defining an Auto (proposed API)
```java
// Defined ONCE in BLineAutos, using factory helpers:
Command middleStandard(BLineAutoFactory f) {
    return Commands.sequence(
        f.pathWithIntake("NO_STOP_MIDDLE_1"),
        f.shootAtHub(),
        f.pathWithImmediateIntake("NO_STOP_MIDDLE_2"),
        f.shootAtHub()
    );
}

// Registration — all 4 variants from one call:
BLineAutoFactory.registerAll(chooser, "Middle Standard", this::middleStandard,
    drivetrain, superStructure, hubShotCalculator, passCalculator);
```

#### `registerAll()` Internally
```
for each Transform (NONE, MIRROR, FLIP, FLIP_MIRROR):
    factory = new BLineAutoFactory(transform, drivetrain, ...)
    label   = "[BLine] " + transform.chooserLabel + " " + autoName
    chooser.addOption(label, autoFn.apply(factory))
```

#### Auto Chooser Labels (mirrors PathPlanner folder structure)
```
[BLine] MASTER Red Right Middle Standard
[BLine] MIRRORED Red Left Middle Standard
[BLine] FLIPPED Blue Right Middle Standard
[BLine] FLIPPED MIRRORED Blue Left Middle Standard
```

### 4. File Structure

```
src/main/java/frc/robot/autos/
├── BLineAutoFactory.java    ← NEW: factory with Transform enum, path loading, registerAll()
├── BLineAutos.java          ← SIMPLIFIED: one method per auto, uses factory API
└── BLinePaths.java          ← DELETED: path geometry lives in files, not code

src/main/deploy/bline/paths/   ← NEW: BLine path files (GUI-edited)
├── NO_STOP_MIDDLE_1.path (or BLine's format)
├── NO_STOP_MIDDLE_2.path
└── ...

scripts/
├── flip_autos.py            ← unchanged (PathPlanner only)
├── mirror_autos.py          ← unchanged (PathPlanner only)
└── generate_all_autos.py    ← unchanged (PathPlanner only)

docs/
├── PathPlannerNamingConventions.md  ← unchanged
└── BLineArchitecture.md             ← NEW: this document
```

### 5. Comparison Table

| Concern              | PathPlanner                          | BLine                              |
|----------------------|--------------------------------------|------------------------------------|
| Path authoring       | PathPlanner GUI → JSON files         | BLine GUI → path files             |
| Path storage         | `deploy/pathplanner/paths/`          | `deploy/bline/paths/` (TBD)        |
| Auto definition      | JSON auto files (GUI)                | Java methods in `BLineAutos`       |
| Variant generation   | Python scripts (offline, file-based) | `BLineAutoFactory` (runtime, Java) |
| Registration         | `AutoBuilder.buildAutoChooser()`     | `BLineAutoFactory.registerAll()`   |
| Transformation math  | Python scripts                       | `BLineAutoFactory.transform()`     |
| Naming pattern       | `MASTER/MIRRORED/FLIPPED/...`        | Same scheme, `[BLine]` tag         |
| Path editing         | PathPlanner GUI                      | BLine GUI                          |

### 6. Key Design Decisions

**Why keep paths as files?**
- GUI editing is faster for iterating trajectory geometry during practice
- Decouples path tuning from code changes (no recompile needed to tweak a path)
- Mirrors PathPlanner's workflow — reduces mental overhead

**Why not Python scripts for BLine variants?**
- BLine paths aren't JSON — they're a different format without an established offline toolchain
- Runtime transformation is simpler and doesn't require a generation step before deploy
- The 4 variants are derived deterministically, so no need to store them

**Why not `Path.flip()` for mirror?**
- BLine's built-in `flip()` handles the X-axis (Red→Blue alliance flip)
- Y-axis mirror (Right side → Left side within same alliance) is a separate transform not built in
- The factory handles both independently and combined

### 7. Migration Path from Current Code
1. Delete `BLinePaths.java` — path geometry moves to GUI-authored files
2. Create `BLineAutoFactory.java` with Transform enum and path loading
3. Rewrite `BLineAutos.java` — replace 4 variants per auto with 1 method + `registerAll()`
4. Verify all 4 chooser entries appear per auto in Shuffleboard/Elastic

---

## Files to Create

| File | Action |
|------|--------|
| `docs/BLineArchitecture.md` | **CREATE** — the architecture document described above |

No code changes in this task — the document describes the target architecture for a future implementation PR.

## Verification

The document is correct if:
- A developer can read it and understand the full factory pattern without reading code
- The comparison table makes PathPlanner vs BLine responsibilities clear
- The proposed API examples are unambiguous enough to implement from
- It is clear that path geometry lives in files, not in Java
