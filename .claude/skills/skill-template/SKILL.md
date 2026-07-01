---
name: skill-template
description: Reference structure for authoring new skills in this repo. Use when writing or reviewing a SKILL.md for a new skill.
user-invocable: false
---

Use this structure when authoring a new skill's `SKILL.md`. Adapt sections to fit — not every skill needs every section (e.g. drop "Prerequisites" if there are none).

```markdown
# [Skill Name] — [Short 3-Letter Code]

[One sentence describing what this skill does and what capability it gives the agent.]

## When to use this skill

[1-2 sentences on when to invoke this skill vs. doing the task manually.]

## Input

`$ARGUMENTS` — [What the user passes, e.g. a file path, match number, log directory, subsystem name. "No arguments required" if none.]

## Prerequisites

[Anything that must exist/be true before running. Delete if none.]

## Steps

### 1. [First step]
[Specific instructions: exact tool, command, file path, or pattern.]

### 2. [Second step]
...

## Output

[What the agent should produce or report: a table, a summary with file paths, a modified file, a pass/fail verdict with evidence.]

## Verification

[Optional: how the agent should verify its own output before reporting, e.g. run the build, replay a log, compare metrics.]

## Example invocation

/[skill-name] [example arguments]
```

## Guidance

- Keep the final `SKILL.md` under 500 lines; move detailed reference material to separate files in the skill directory.
- Keep the body concise — once invoked, it stays loaded in context for the rest of the session, so every line is a recurring token cost. State what to do, not how or why.
- Skills live at `.claude/skills/<skill-name>/SKILL.md` (project) or `~/.claude/skills/<skill-name>/SKILL.md` (personal). The directory name becomes the `/command` name.

*Skill design principle (from FRC 254's "The Next Revolution: AI in FRC"):*
*Skills give agents access to your robot's debugging tools. The minimum useful agent loop is: read logs → replay or simulate → compare results → iterate. Each skill should own one step of that loop. A well-scoped skill lets the agent accomplish a specific task reliably — a poorly scoped skill produces unpredictable results.*
