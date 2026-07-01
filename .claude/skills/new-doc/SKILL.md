---
name: new-doc
description: Create a new documentation file in docs/ following this repo's standardized templates for advanced_topic, core_architecture, and implementation_plan doc types. Use when the user asks to write/create new documentation, a design doc, an architecture guide, or an implementation plan.
argument-hint: [type: advanced_topic|core_architecture|implementation_plan] [title]
arguments: [type, title]
---

Create a new doc under `docs/` matching one of this repo's three existing doc types. Do not invent a fourth structure — pick the closest of the three below. If `$type` is missing or doesn't match one of the three, ask the user which type fits before writing anything.

## Choosing type and location

| Type | When to use | Directory | Filename pattern | Example |
|---|---|---|---|---|
| `advanced_topic` | Deep dive on a single algorithm/subsystem's math or control theory (equations, physics, tuning theory) | `docs/advanced_topics/` | `PascalCase.md` | `ShotCalculatorEquations.md` |
| `core_architecture` | Explains a structural pattern used across the codebase (how subsystems/IO/state machines are organized) | `docs/core_archictecture/` (keep the existing typo — renaming breaks links) | `PascalCase.md` | `SubsystemArchitecture.md` |
| `implementation_plan` | A concrete plan to implement a specific feature/issue, with file-by-file changes | `docs/implementation_plans/` | `kebab-case-implementation-plan.md`, reference the issue number in the title if one exists | `alerts-implementation-plan.md` |

Before writing, grep the relevant source files (`Grep`/`Glob`) so the doc's code examples and API references are accurate to the current codebase, not guessed.

## Template: advanced_topic

```markdown
# [Topic Name]

[One sentence: what this document explains and which source file(s) it documents, e.g. "This document visualizes the equations used in `X.java` for Y."]

---

## 1. [First concept/equation]

[Equation in LaTeX ($$...$$) or code, followed by a "Where:" list defining each symbol with units.]

## 2. [Next concept]
...

[Continue numbered sections, each one equation/step, each followed by a "Where:" definitions list and, where non-obvious, a **Physical meaning:** line explaining why the math is shaped this way.]

---

## Complete Data Flow Diagram

[ASCII diagram showing how the inputs flow through the numbered steps to the outputs.]

---

## Coordinate Frames

[Only if spatial: define reference frames, origin, rotation direction.]

## Units

| Variable | Unit |
|----------|------|
| ... | ... |
```

## Template: core_architecture

```markdown
# [Pattern Name] Guide

[One paragraph: what structural pattern this explains and why it exists.]

---

## Table of Contents

1. [Overview](#overview)
2. [section links matching the headers below]

---

## Overview

[ASCII layered diagram of the pattern, plus attribution if adapted from a known FRC team's architecture.]

---

## [Pattern Section 1, e.g. "The IO Layer Pattern"]

### Purpose
[Why this piece exists.]

### Components
[Real code examples pulled from the actual source files — interface definition, then a real vs. sim implementation side by side.]

## [Pattern Section 2, e.g. "State Machine Pattern"]
[Same structure: purpose, then real code from the codebase.]

... [Repeat one section per structural concept in the pattern.]

---

## Complete Code Structure

[One full annotated example file showing every section a file of this type contains, with `// ========== SECTION ==========` banner comments.]

### Code Organization Rules

| Section | Contents |
| --- | --- |
| ... | ... |

---

## Data Flow

[ASCII diagram of the runtime call sequence, e.g. periodic() loop order.]

---

## File Organization

[Directory tree of where these files live, plus a naming conventions table: Pattern | Usage | Example.]

---

## Key Principles

[Short numbered list of the design principles this pattern enforces, e.g. Single Responsibility, Dependency Inversion.]

## Common Patterns

[Named sub-patterns worth calling out individually, each with a short code snippet — e.g. "Pattern: Supplier-Based Dynamic Setpoints".]

---

## Summary

| Layer/Component | Responsibility | Files |
| --- | --- | --- |
| ... | ... | ... |
```

## Template: implementation_plan

```markdown
# [Feature Name] Implementation Plan (Issue #[number])

## Goal

[What this implements and why, in 1-3 sentences. If the source issue lists distinct pillars/requirements, enumerate them.]

---

## [API/Pattern Reference section(s), one per relevant vendor library or existing code pattern]

[e.g. "## WPILib Alert API (existing pattern from `X.java`)" — show the existing pattern being extended, with a real code snippet from the codebase.]

### [Hardware/Library Variant, e.g. "TalonFX (CTRE Phoenix 6)"]
Used by: [list of subsystems]

#### [Sub-capability, e.g. "Connectivity check"]
[Exact API call, code snippet, and any caveats — bus traffic cost, update frequency, gotchas found while researching.]

... [Repeat per hardware variant / library used across the affected subsystems.]

---

## [Conditions/Detection table, one per hardware variant]

| Alert/Condition | Detection Method |
|---|---|

---

## Implementation Plan

### Step 1 — [short verb phrase]

[Instructions, followed by a file-change table:]

| File | Change | Notes |
|---|---|---|

### Step 2 — [short verb phrase]
...

[Repeat one numbered step per logical unit of work. Each step should be independently completable/testable.]

---

## [Tunable Values table, if applicable]

| Subsystem | Value | Threshold |
|---|---|---|

---

## File Change Summary

| File | Change |
|---|---|

---

## Implementation Order

[Numbered recommended sequence — easiest/lowest-risk first, so partial progress is always mergeable. Explain briefly why this order.]
```

## After writing

- Save to the path determined above.
- Report the file path and a one-line summary of what was written back to the user.
- Do not add the new doc to any index/README unless one already exists and already lists sibling docs of the same type.
