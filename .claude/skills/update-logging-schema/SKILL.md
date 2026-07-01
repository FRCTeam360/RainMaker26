---
name: update-logging-schema
description: Audit and update docs/LoggingOrganization.md so it matches the actual Logger.recordOutput()/Logger.processInputs() calls in source. Use when the user asks to update, sync, or audit the logging schema/keys, or after adding/renaming/removing AdvantageKit log calls.
argument-hint: [optional: subsystem or file name to scope the audit]
---

`docs/LoggingOrganization.md` is a hand-maintained reference of every AdvantageKit `Logger.recordOutput()` key and `Logger.processInputs()` namespace. It drifts from the code because nothing enforces it. This skill reconciles the doc against the actual source rather than trusting either one blindly.

## Steps

### 1. Extract actual keys from source

Grep `src/main/java` for `Logger.recordOutput(` and `Logger.processInputs(`. For each call:

- If the key is a string literal, record it verbatim along with the file and line.
- If the key is built dynamically (string concatenation, a variable, a helper like `blineKey(...)`, an f-string-equivalent), record the containing file/method and the static prefix you can determine (e.g. `"BLineAutos/MissingPaths/" + autoName` → prefix `BLineAutos/MissingPaths/<dynamic>`). Do not guess the runtime value — mark it dynamic.
- Note `Logger.processInputs("X", inputs)` calls separately — these log an entire `@AutoLog` inputs class under `RealInputs/X` (per the existing note in the doc), not an individual key.

Use `Grep` with `-n` and `output_mode: content`, scoped to `$ARGUMENTS` if a subsystem/file was given, otherwise the whole `src/main/java` tree.

### 2. Diff against the existing doc

Read `docs/LoggingOrganization.md`. For every key found in step 1:

- **Missing from doc** → needs to be added.
- **Present in doc but not found in source** → the source key was likely renamed or removed; flag it, don't silently delete without confirming the source truly no longer produces it (check for the old key elsewhere first, e.g. renamed variable).
- **Present in both** → leave as-is unless the `Source` file column is now wrong (moved to a different class).

### 3. Place new keys correctly

Match the existing file's structure and conventions section:

- Find the right top-level section (`Robot/`, `Swerve/`, `Vision/`, `Superstructure/`, `Utils/`, `Commands/`, etc.) by the key's first path segment. Create a new `## <Segment>/` section only if no existing section fits — don't force an unrelated key into an existing section.
- Nest under the matching subsection (e.g. new subsystem keys go under `Superstructure/Subsystems/<Name>/`, new state machine keys under `Superstructure/StateMachines/<Name>/`).
- Follow the conventions already documented at the bottom of the file: CamelCase segments, capitalized first letter, units baked into the name (`...Seconds`, `...Degrees`, `...Meters`), no spaces.
- Add a `Source` column entry with the exact file name, and a short `Description` only when the key name doesn't already make the value obvious (match the terseness of existing rows — most rows have an empty description).

### 4. Handle dynamic keys

For keys built dynamically, add one row using the static prefix and a note in the Description column, e.g. `Autos/BLineAutos/MissingPaths/<autoName>` with description "One entry per failed auto name". Don't enumerate runtime values.

### 5. Write the update

Edit `docs/LoggingOrganization.md` directly, preserving table formatting (pipe-aligned isn't required — match whatever the existing file does, currently unaligned pipes) and the existing section order. Do not reorder or rewrite sections that had no changes.

## Output

Report back:
- Count of keys added, and which sections they landed in.
- Any keys flagged as possibly-stale (in doc, not found in source) — list them for the user to confirm before removal; do not remove them yourself without confirmation.
- Any dynamic-key call sites that needed a judgment call on the documented prefix.

## Notes

- This is a docs-only change — never modify `Logger.recordOutput`/`Logger.processInputs` call sites themselves under this skill.
- If the user asks whether the schema should move off markdown into a structured/generated format: no migration is warranted for a single-consumer human reference doc. Only reconsider if the team wants build-time enforcement (e.g. failing CI on undocumented keys), which is a separate, larger decision.
