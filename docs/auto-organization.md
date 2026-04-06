# Auto Organization — RainMaker26 (2026 REBUILT)

## Overview

The robot runs two parallel autonomous systems for A/B testing: **PathPlanner** (file-based) and **BLine** (programmatic, in-code). Both use the same spatial organization and naming conventions. All routines are authored once as **MASTER** autos for the **Red Right** starting position, then generated or transformed for all other alliance/side combinations.

---

## Folder Structure

### PathPlanner Autos

PathPlanner organizes autos into four folders under `src/main/deploy/autos/`:

| Folder | Alliance | Side | How it's made |
|---|---|---|---|
| `MASTER RED RIGHT` | Red | Right | Hand-authored in PathPlanner |
| `RED LEFT AUTOS` | Red | Left | Mirrored from MASTER via script |
| `BLUE RIGHT AUTOS` | Blue | Right | Flipped from MASTER via script |
| `BLUE LEFT AUTOS` | Blue | Left | Flipped from RED LEFT via script |

Each auto's referenced paths live in a PathPlanner path folder named after the auto (e.g., `FLIPPED Blue Right Middle Hook`). This avoids name collisions between variants.

### BLine Autos

BLine paths are defined programmatically in [BLineAutos.java](../src/main/java/frc/robot/autos/BLineAutos.java). Master coordinates are defined once for Red Right, then transformed into all four variants using coordinate math methods in the same file. All BLine autos are registered in the auto chooser with a `[BLine]` prefix so they appear alongside PathPlanner autos.

---

## Coordinate Geometry

**Mirror** — reflects across the field's horizontal centerline (same alliance, opposite side):
```
new_x = old_x
new_y = field_width - old_y      (field_width = 8.07 m)
new_rotation = -rotation
```

**Flip** — 180° rotation around the field center (opposite alliance, same relative side):
```
new_x = field_length - old_x     (field_length = 16.54 m)
new_y = field_width  - old_y
new_rotation = rotation + 180°
```

> "Same side" after a flip is because both alliance drivers face inward — Red's right and Blue's right are the same physical side of the field.

**Flip + Mirror** — flip then mirror (Red Right → Blue Left):
```
new_x = field_length - old_x
new_y = old_y
new_rotation = -(rotation + 180°)
```

---

## Naming Convention

### PathPlanner Autos

| Prefix | Meaning |
|---|---|
| `MASTER` | Hand-authored source auto (Red Right) |
| `MIRRORED` | Generated — left/right mirror of a MASTER |
| `FLIPPED` | Generated — 180° flip of a MASTER |
| `FLIPPED MIRRORED` | Generated — 180° flip of a MIRRORED auto |

`MASTER` is stripped from generated file names. Examples:

- `MASTER Red Right Middle Hook` → mirror → `MIRRORED Red Left Middle Hook`
- `MASTER Red Right Middle Hook` → flip → `FLIPPED Blue Right Middle Hook`
- `MIRRORED Red Left Middle Hook` → flip → `FLIPPED MIRRORED Blue Left Middle Hook`

### BLine Autos (Auto Chooser Labels)

BLine autos follow the same naming convention with a `[BLine]` prefix:

| Chooser Label | Variant |
|---|---|
| `[BLine] MASTER Red Right Middle Standard` | MASTER |
| `[BLine] MASTER Red Right Middle Hawk` | MASTER |
| `[BLine] MASTER Red Right Middle Hook` | MASTER |
| `[BLine] MASTER Red Right Depot` | MASTER |
| `[BLine] MASTER Red Right Trench Middle` | MASTER |
| `[BLine] MIRRORED Red Left Middle Standard` | MIRRORED |
| `[BLine] MIRRORED Red Left Middle Hawk` | MIRRORED |
| `[BLine] MIRRORED Red Left Middle Hook` | MIRRORED |
| `[BLine] FLIPPED Blue Right Middle Standard` | FLIPPED |
| `[BLine] FLIPPED Blue Right Middle Hawk` | FLIPPED |
| `[BLine] FLIPPED Blue Right Middle Hook` | FLIPPED |
| `[BLine] FLIPPED MIRRORED Blue Left Middle Standard` | FLIPPED MIRRORED |
| `[BLine] FLIPPED MIRRORED Blue Left Middle Hawk` | FLIPPED MIRRORED |
| `[BLine] FLIPPED MIRRORED Blue Left Middle Hook` | FLIPPED MIRRORED |

GUI-designed paths (used for testing) are also registered inline directly with `new Path("filename")`.

---

## Scripts

All scripts live in `scripts/` and require only the Python standard library.

### `generate_all_autos.py` — recommended entry point

Generates all three variants (MIRRORED, FLIPPED, FLIPPED MIRRORED) from a single MASTER auto in one command:

```
python scripts/generate_all_autos.py "MASTER Red Right Middle Hook"
python scripts/generate_all_autos.py --dry-run "MASTER Red Right Middle Hook"
```

Internally runs mirror then flip steps in sequence:
1. Mirror `MASTER Red Right` → `RED LEFT AUTOS`
2. Flip `MASTER Red Right` → `BLUE RIGHT AUTOS`
3. Flip the mirrored result → `BLUE LEFT AUTOS`

### `mirror_autos.py` — mirror only

Reflects across the field's Y centerline (Red Right → Red Left):

```
python scripts/mirror_autos.py "MASTER Red Right Middle Hook"
```

Output goes to `RED LEFT AUTOS/`.

### `flip_autos.py` — flip only

180° rotation around field center (Red → Blue, or Left → Right):

```
python scripts/flip_autos.py "MASTER Red Right Middle Hook"
python scripts/flip_autos.py "MIRRORED Red Left Middle Hook"
```

Output goes to `BLUE RIGHT AUTOS/` or `BLUE LEFT AUTOS/` respectively.

### `audit_paths.py` — find unused paths

Reports which path files are not referenced by any auto, and which autos reference missing paths:

```
python scripts/audit_paths.py
python scripts/audit_paths.py --all       # show all paths, not just orphans
python scripts/audit_paths.py --delete    # delete unreferenced path files
```

---

## BLine Coordinate Transformation (Java)

In [BLineAutos.java](../src/main/java/frc/robot/autos/BLineAutos.java), master paths are defined for Red Right, then four helpers apply the same math as the Python scripts:

| Method | Transform | Direction |
|---|---|---|
| `mirrorY(t)` | `(x, width - y)` | Red Right → Red Left (translation) |
| `mirrorRotation(r)` | `-rotation` | Red Right → Red Left (rotation) |
| `flipX(t)` | `(length - x, y)` | Red → Blue (translation X only) |
| `flipRotation(r)` | `rotation + 180°` | Red → Blue (rotation) |
| `flipAndMirror(t)` | `flipX` then `mirrorY` | Red Right → Blue Left (translation) |
| `flipAndMirrorRotation(r)` | `flipRotation` then `mirrorRotation` | Red Right → Blue Left (rotation) |

Each path variant family (MIRRORED, FLIPPED, FLIPPED MIRRORED) has its own set of private path factory methods that apply these transforms to every waypoint, translation target, and rotation target.

---

## Workflow

### Adding a new PathPlanner auto

1. Author the auto in PathPlanner as a **MASTER Red Right** auto and place it in the `MASTER RED RIGHT` folder.
2. Run `generate_all_autos.py` with the auto name.
3. All three variants appear in PathPlanner under their respective folders.
4. To regenerate after editing a MASTER, delete the three generated autos and their path folders, then re-run the script.

> Do not hand-edit generated autos or their paths — changes will be overwritten on the next generation.

### Adding a new BLine auto

1. Define MASTER (Red Right) path factory methods in `BLineAutos.java`.
2. Add MIRRORED, FLIPPED, and FLIPPED MIRRORED path factory methods applying the coordinate transform helpers.
3. Add MASTER, MIRRORED, FLIPPED, and FLIPPED MIRRORED auto command methods.
4. Register all four variants in `registerAutos()` with the correct `[BLine] <VARIANT> <Alliance> <Side> <Name>` label.
