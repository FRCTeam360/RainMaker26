---
name: docs-index
description: Index of every doc in docs/ (advanced_topics, core_architecture, implementation_plans, top-level, ai-usage) with a one-line topic each. Use when asked how something works, where something is documented, whether a doc already exists on a topic, or before writing a new doc.
paths: ["src/main/java/frc/robot/**"]
---

Full-text docs are expensive to keep in context. This index is not. Find the relevant row below, then `Read` only that file — don't paste doc contents here, and don't re-summarize docs you haven't opened.

## advanced_topics/ — math, control theory, algorithm deep-dives

| Doc | Topic |
|---|---|
| `docs/advanced_topics/ShotCalculatorEquations.md` | Full equation-by-equation breakdown of `ShotCalculator.java` (lookahead, interpolation, target heading) |
| `docs/advanced_topics/BangBangFlywheelControl.md` | Why/how the flywheel uses bang-bang control instead of PID for rapid-fire shooting |
| `docs/advanced_topics/ShootOnTheMove.md` | Findings & optimization roadmap for the shoot-on-the-move `ShotCalculator` (adapted from FRC 6328) |
| `docs/advanced_topics/PhotonVisionSimGuide.md` | How PhotonVision simulation integrates with the AdvantageKit IO pattern |

## core_archictecture/ — structural patterns used across the codebase

| Doc | Topic |
|---|---|
| `docs/core_archictecture/SubsystemArchitecture.md` | IO layer pattern, state machine pattern, dual control modes, SuperStructure coordination — the primary architecture reference |
| `docs/core_archictecture/SubsystemPatterns.md` | Common patterns across all `SubsystemBase` classes (companion to SubsystemArchitecture/AdvantageKit guides) |
| `docs/core_archictecture/AdvantageKitSubsystemGuide.md` | How to build a subsystem with AdvantageKit's three-layer architecture + dependency injection |
| `docs/core_archictecture/StateMachineDiagramGuide.md` | Standard conventions for drawing this repo's state machine diagrams |
| `docs/core_archictecture/SimulationArchitecture.md` | Simulation architecture for both CTRE (TalonFX) and REV (SparkMax) motor controllers |
| `docs/core_archictecture/HubShiftMechanics.md` | How active-hub tracking and the TOF/sensor delay offset work in teleop |

## implementation_plans/ — concrete plans for a specific feature/issue

| Doc | Topic |
|---|---|
| `docs/implementation_plans/alerts-implementation-plan.md` | Hardware health alerts (connectivity, stall, temperature) for motor/sensor subsystems (#545) |
| `docs/implementation_plans/BLineArchitecture.md` | Plan for BLine (in-code polyline autonomous) vs. PathPlanner architecture |
| `docs/implementation_plans/heading-controller-defense-resistance.md` | Making the heading controller resistant to opponents physically spinning the robot while shooting |

## Top-level docs/

| Doc | Topic |
|---|---|
| `docs/auto-organization.md` | How PathPlanner and BLine autos are organized, named, and generated across alliance/side variants |
| `docs/LoggingOrganization.md` | Full key hierarchy for every `Logger.recordOutput()` call — see also the `update-logging-schema` skill |

## ai-usage/

| Doc | Topic |
|---|---|
| `docs/ai-usage/team360-ai-summary.md` | Season summary of AI-assisted development on the team |
| `docs/ai-usage/2026-03-02-claude-code-review-bug-report.md` | Audit of real bugs caught by Claude code review across merged PRs |

## Diagrams (not markdown — reference only, don't read as text)

`.mmd` (Mermaid source) / `.png` / `.svg` under `docs/diagrams/`: architecture diagrams (`docs/diagrams/architecture/`), full/subsystem state machines (`docs/diagrams/state_machines/`, `docs/diagrams/subsystem_states/`). Regenerate rendered output with `docs/diagrams/convert-diagrams.py` after editing a `.mmd` source.

## Keeping this index current

This file is a manually maintained list — it will drift if a doc is added, renamed, or moved (like `new-doc` and `robot-config-parity-check`) without this table being updated. When you use `new-doc` to create a doc, or move/rename a doc, add/update its row here in the same change.
