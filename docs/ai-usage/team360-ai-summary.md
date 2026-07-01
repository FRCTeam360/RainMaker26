# AI-Assisted Development on FRC Team 360 — 2026 Summary

**Team:** FRC 360 | The Revolution | PNW District | Season: 2026 (REEFSCAPE)
**Repo:** [github.com/FRCTeam360/RainMaker26](https://github.com/FRCTeam360/RainMaker26)

---

## Why We Built This System

Writing code is not the bottleneck for a small team. PR review turnaround, students learning new concepts, and robot access time are the actual slowdowns. We built an AI-assisted workflow to accelerate review and reduce mentor time spent on routine checks — not to replace the coding work students need to do to grow.

---

## The Three-Layer Review System

| Layer | Tool | Role |
|---|---|---|
| CI Pipeline | Gradle, Spotless, WPILib Sim | Automated floor — does it compile, format, and run? |
| AI First-Pass | Claude (GitHub integration) | Catches bugs, convention violations, and null risks before mentor review |
| Mentor Review | Human leads | Architecture decisions, field-specific judgment, teaching moments |

The CI pipeline compounds before AI is ever involved. Spotless auto-format on every `compileJava` call eliminated formatting as a category of review overhead entirely. The sim test workflow catches command lifecycle and API misuse at runtime. AI first-pass then handles the layer that was previously eating mentor time: structural and logic bugs that are fast to describe but require reading code to find.

---

## CI Pipeline (9 GitHub Actions Workflows)

### Merge-blocking

| Workflow | What it does |
|---|---|
| `build.yml` | Gradle build — does it compile? |
| `format-check.yml` | `spotlessCheck` — formatting as a hard gate |
| `sim-test.yml` | Runs code in WPILib headless simulation; fails on "Error at" / "Exception" |
| `test.yml` | `./gradlew test` — unit tests |

### Advisory (non-blocking)

| Workflow | What it does |
|---|---|
| `spotbugs.yml` | Static bug detection; used to seed good-first-issues for new students |
| `claude-code-review.yml` | Auto-triggers Claude review on Java / `build.gradle` changes |
| `claude.yml` | `@claude` mentions for self-service follow-up reviews |
| `rewrite.yml` | OpenRewrite dry run for code modernization preview |
| `python-test.yml` | Runs `scripts/tests/test_*.py` |

---

## Claude Code Review Integration

**How it works:**
1. PR opened with Java or `build.gradle` changes → `claude-code-review.yml` queues
2. CI checks for an existing review to avoid duplicates on iterating PRs
3. Claude receives the diff + HEAD SHA via `gh api`, reviews against `CLAUDE.md` guidelines
4. Single comprehensive review posted: inline comments + PR-level summary
5. Contributors can trigger follow-ups via `@claude` with a specific focus angle

The review checklist in `CLAUDE.md` is tuned to break-only: command lifecycle changes, CAN ID conflicts, disabled safety checks, null risks. It explicitly skips style and naming unless they affect correctness.

**Tuning notes:**
- Originally triggered on every push — produced 114-comment floods on iterating students. Scoped to PR open only.
- Checklist tightened toward breaking changes as competition deadlines approached.
- Feeding stale diffs was an early issue; CI now checks for prior reviews before posting.

**Evidence of real impact:** As of 2026-03-02, Claude caught 35+ real bugs across 15+ PRs — missing `break` statements (dead states), infinite recursion at construction, disabled NT4Publisher (no telemetry on hardware), disabled stator current limits, and null crashes before match start. Bug report: `docs/ai-usage/2026-03-02-claude-code-review-bug-report.md`.

---

## Skills / Claude Commands

Skills (`.claude/commands/*.md`) give Claude agents a toolbox for specific tasks. Rather than describing everything in one monolithic prompt, each skill documents one repeatable capability: what it does, what to pass, what to produce.

FRC 254's framing from "The Next Revolution: AI in FRC" (2026): skills give agents access to the robot's debugging environment. The minimum useful agent loop is **read logs → replay or simulate → compare results → iterate**. Each skill owns one step of that loop.

### Skills in this repo

| File | Purpose |
|---|---|
| `fsm.md` | Implement a new state machine for a subsystem — generates the full FSM scaffold with IO layer, inputs class, implementations, and an HTML state diagram |
| `SKILL_TEMPLATE.md` | Blank skeleton for creating new skills — starting point, not a guide |

**`fsm.md` was built by JRTaylord.** It is the most developed skill in the repo and a good reference for what a well-scoped skill looks like: it takes a subsystem name as input, produces concrete files, and includes HTML output for visualization.

### Skills recommended by FRC 254 (not yet implemented)

254's presentation recommends organizing skills in a `SKILLS/` directory by type:

| Type | Description |
|---|---|
| `SIM` | Simulation agent — starts, configures, runs sim and captures output |
| `BOT` | Robot description — explains the robot's physical and software structure |
| `REP` | Replay testing — feeds a match log through sim to reproduce a behavior |
| `LOG` | Log reading — parses and summarizes `.wpilog` files for a specific signal or match |
| `GAME` | Game information — explains field elements, rules, scoring relevant to code decisions |

We don't have these yet. The closed-loop debugging workflow (LOG → REP/SIM → compare → iterate) is the intended payoff — a robot diagnostic loop the agent can run without a human doing each step manually. `fetch_log.py` already handles the log retrieval step if SSH access is available.

---

## GitHub Issues Automation

Used Claude Code to scan the codebase and suggest good-first-issues for new students — specifically SpotBugs findings, naming convention violations, and uninitialized subsystem states. Mentors curated the final issue list for clarity and approachability.

**What worked:** SpotBugs generates more findings than mentors have time to triage into teaching moments. AI-seeded issues let mentors review a pre-filtered list and turn technical findings into structured onboarding tasks.

**What didn't:** AI-generated issue creation for ongoing project management (not just student issues) produced duplicates and added overhead rather than reducing it. Reverted to manual issue management for the sprint board.

---

## Student AI Access Policy

- A subset of students were given OpenCode (multi-model AI coding assistant) based on mentor assessment of programming maturity and curiosity.
- Only ~2 of 10 students use it regularly. Most are appropriately cautious about trusting AI-generated robot code.
- AI is not pushed on students who don't want it.
- **Programming 1 students** (first-year, post-Java assessment) work initial issues without AI to build foundational skills. Two explicit exceptions: documenting code they didn't write, and reorganizing code for consistency — both are considered low-engagement tasks not worth using as learning time.
- Most AI-generated code in the repo lives in sim infrastructure, build process scripts, and mentor-contributed code with direct oversight. Students are not generating robot subsystems, commands, or state machines with AI.

---

## Cost

| Tool | Cost | Notes |
|---|---|---|
| Claude Max | $200/month (mentor personal) | GitHub code review workflow + mentor dev use; $100/month tier likely sufficient |
| OpenCode Zen | $25 (team) | Multi-model AI access for student experimentation |
| Gemini GitHub integration | Free | Used by 2412; considered as comparison/fallback |

---

## What Worked vs. What Didn't

**Worked:**
- CI pipeline compounding before AI is added — formatting and build checks are cheap wins that reduce review noise on their own
- AI code review as first-pass triage, not a gate — blocked zero PRs, caught real bugs
- Using AI as a learning accelerant for mentors: deep research on patterns, control theory questions, architecture decisions before bringing to students
- GitHub Issues seeding for new student onboarding
- Simulation setup assisted with AI getting 80% of the way there

**Didn't work:**
- AI-assisted sprint/project management issue creation — too many duplicates
- Blanket AI access for students — most don't want it and the ones who do need calibration time

**Still open:**
- Skills (LOG, REP, SIM) for closed-loop robot debugging — the biggest unrealized value in the 254 framework for us
- Scoping SpotBugs to diff-only analysis to make it a merge gate instead of just a health check

---

## Community Context

This system draws from what multiple top teams have published:

| Team | What we borrowed |
|---|---|
| 6328 (Mechanical Advantage) | AdvantageKit IO architecture, Spotless auto-format on build |
| 2412 (Robototes) | Sim test workflow concept, Gemini AI review as comparison point |
| 254 (Cheesy Poofs) | ControlState pattern; "The Next Revolution: AI in FRC" framework for skill design |
| 2910 | Superstructure-based control system architecture |

254's presentation is the most useful reference for anyone thinking about skills specifically: it distinguishes surgical AI use (precise targeted edits) from spec-driven development (Goal → Plan → Review → Implement → Verify) from vibe coding, and argues spec-driven is where teams get the most value for safety-critical robot code. The presentation is available publicly from Chief Delphi.

---

*Last updated: 2026-06-30*
