# Auto Organization — RainMaker26 (2026 REBUILT)

## Overview

All autonomous routines are authored once as **MASTER** autos for the **Red Right** starting position, then automatically generated for all other alliance/side combinations using scripts in the `scripts/` directory.

## Folder Structure

PathPlanner organizes autos into four folders:

| Folder | Alliance | Side | How it's made |
|---|---|---|---|
| `MASTER RED RIGHT` | Red | Right | Hand-authored in PathPlanner |
| `RED LEFT AUTOS` | Red | Left | Mirrored from MASTER |
| `BLUE RIGHT AUTOS` | Blue | Right | Flipped from MASTER |
| `BLUE LEFT AUTOS` | Blue | Left | Flipped from RED LEFT (MIRRORED) |

## Coordinate Geometry

**Mirror** — reflects across the field's horizontal centerline (same alliance, opposite side):
```
new_x = old_x
new_y = field_width - old_y
new_rotation = -rotation
```

**Flip** — 180° rotation around the field center (opposite alliance, same side):
```
new_x = field_length - old_x
new_y = field_width  - old_y
new_rotation = rotation + 180°
```

> "Same side" after a flip is because both alliance drivers face inward — Red's right and Blue's right are the same physical side of the field.

## Naming Convention

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

## Path Folders

Each generated auto's referenced paths are placed in a PathPlanner path folder named after the generated auto (e.g. `FLIPPED Blue Right Middle Hook`). This keeps paths organized and avoids name collisions across variants.

## Scripts

All scripts live in `scripts/` and require only the Python standard library.

### `generate_all_autos.py` — recommended entry point

Generates all three variants of a MASTER auto in one command:

```
python scripts/generate_all_autos.py "MASTER Red Right Middle Hook"
python scripts/generate_all_autos.py --dry-run "MASTER Red Right Middle Hook"
```

### `mirror_autos.py` — mirror only

```
python scripts/mirror_autos.py "MASTER Red Right Middle Hook"
```

### `flip_autos.py` — flip only

```
python scripts/flip_autos.py "MASTER Red Right Middle Hook"
python scripts/flip_autos.py "MIRRORED Red Left Middle Hook"
```

## Workflow

1. Author the auto in PathPlanner as a **MASTER Red Right** auto and place it in the `MASTER RED RIGHT` folder.
2. Run `generate_all_autos.py` with the auto name.
3. All three variants appear in PathPlanner under their respective folders.
4. To regenerate after editing a MASTER, delete the three generated autos and their path folders, then re-run the script.

> Do not hand-edit generated autos or their paths — changes will be overwritten on the next generation.
