---
name: pr-review
description: Review scope and breaking-change filtering rules for reviewing pull requests in this repo. Use whenever reviewing a PR or diff (via /code-review, /review, or manual review) — not for writing new code.
---

Apply these when reviewing a PR or diff in this repo. This is review-mode scoping only — the code quality checklist itself lives in CLAUDE.md under "Code Review Checklist" and applies whether you're generating or reviewing code; this skill tells you how to *filter* that checklist during review.

## Review scope and rules

- **Focus on breaking changes, not nitpicks.** Flag things that could break the robot, change runtime behavior unexpectedly, or affect other robot configs (SIM, WoodBot, PracticeBot). Do not nitpick style, naming, or minor conventions — trust the author.
- **Only review lines that are part of the PR diff.** Do not flag pre-existing issues in files the PR touches. If a line was not added or modified in the diff, do not comment on it.
- **Focus on correctness and intent of the PR.** Evaluate whether the changed code does what the PR description says and doesn't introduce new issues. Do not expand scope beyond the PR's purpose.
- **CI status:** Only Build, Test, Format Check, and Simulation Test are merge blockers.
- **Code Review Checklist (in CLAUDE.md):** only flag a violation if it represents a breaking change or correctness issue — not as a standalone nitpick.

## What counts as a breaking change

Flag these — they can break the robot or other configs:

- **Behavioral changes**: command lifecycle changes (e.g., `runEnd` to `runOnce`), removed cleanup/stop actions, changed motor inversions or sensor polarity
- **Cross-config impact**: changes to shared code (SuperStructure, RobotContainer bindings, command factories) that affect configs other than the one being modified
- **Hardcoded sensor overrides**: stubbing out sensor values (e.g., `sensorActivated = true`) that gate state transitions or safety logic
- **CAN ID conflicts**: two constants sharing the same ID on the same bus — see the `can-id-audit` skill
- **Disabled functionality**: commenting out periodic calculations, safety checks, or sensor reads that other code depends on
- **API signature changes**: renamed or removed public methods that may have callers outside the diff
- **Subsystem parity**: a robot config left without a required subsystem/Noop IO — see the `robot-config-parity-check` skill

## What is NOT worth flagging

Do not comment on these unless they cause a breaking change:

- Style, formatting, or naming preferences
- Missing Javadoc or comments
- Unnamed numeric literals in test/tuning bindings
- Code organization or method extraction choices
- Conventions from the CLAUDE.md checklist that don't affect correctness

## Review-only check

- [ ] No NI Driver Station artifacts in source files — the DS enable shortcut inserts `\[]` into the focused window, and pressing Enter to disable can insert stray newlines. Flag any `\[]` occurrences or unexpected blank lines in code/comments that may have been typed while the DS was focused. (Forensic check on existing diffs only — not applicable to code generation.)
