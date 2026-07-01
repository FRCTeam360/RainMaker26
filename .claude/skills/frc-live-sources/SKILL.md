---
name: frc-live-sources
description: Curated WebFetch URLs for WPILib, CTRE Phoenix 6, REVLib, PathPlannerLib, and AdvantageKit docs — mirrors the vendor dependency table in CLAUDE.md but one level deeper (topic-specific pages, not just doc-site roots). Use when you need current/authoritative info on a WPILib/CTRE/REV/PathPlanner/AdvantageKit API, behavior, or setup step instead of guessing a URL or relying on training-data knowledge that may be stale for this season's release.
---

Vendor libraries update yearly (new WPILib season release, CTRE/REV major version bumps). Don't guess a deep doc URL from memory — fetch a known-good one from this table. If a listed URL 404s (moved/renamed), fall back to the doc site's root from the CLAUDE.md vendor table and search from there, then update the row below.

## When to use

- The user asks "how do I do X in WPILib/Phoenix6/REVLib/PathPlanner/AdvantageKit" and the answer depends on exact current API shape (method names, config classes, units).
- You're about to write vendor-library code and aren't fully confident of the current API surface from training data alone.
- Not for questions answerable from this repo's own code/docs — check `docs-index` first.

## WPILib

| Topic | URL |
|---|---|
| Docs root | `https://docs.wpilib.org/en/stable/` |
| Kinematics & odometry (swerve/differential) | `https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html` |
| Trajectories & path following | `https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html` |
| PID control | `https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html` |
| Command-based programming | `https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html` |
| SysId characterization | `https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html` |
| Simulation | `https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html` |
| Java API (Javadoc) | `https://github.wpilib.org/allwpilib/docs/release/java/index.html` |

## CTRE Phoenix 6

| Topic | URL |
|---|---|
| Docs root | `https://v6.docs.ctr-electronics.com/en/stable/` |
| Swerve (Tuner X / TunerConstants) | `https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html` |
| TalonFX configuration | `https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/index.html` |
| CAN bus / Pro licensing | `https://v6.docs.ctr-electronics.com/en/stable/docs/canivore/canivore-migration.html` |
| Java Javadoc | `https://api.ctr-electronics.com/phoenix6/stable/java/index.html` |

## REVLib

| Topic | URL |
|---|---|
| Docs root | `https://docs.revrobotics.com/revlib` |
| SparkMax/SparkFlex configuration | `https://docs.revrobotics.com/revlib/spark/configuring-spark` |
| Java Javadoc | `https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html` |

## PathPlannerLib

| Topic | URL |
|---|---|
| Docs root | `https://pathplanner.dev/home.html` |
| Named commands (autos) | `https://pathplanner.dev/pplib-named-commands.html` |
| Java API | `https://pathplanner.dev/api/java/` |

## AdvantageKit

| Topic | URL |
|---|---|
| Docs root | `https://docs.advantagekit.org/` |
| IO layer / logged inputs pattern | `https://docs.advantagekit.org/data-flow/recording-inputs/` |
| Java Javadoc | `https://docs.advantagekit.org/javadoc/` |

## After fetching

Extract only what's needed for the current task — don't paste whole doc pages into the conversation. If the fetched page contradicts something in this repo's CLAUDE.md or a `docs-index` entry, flag the discrepancy rather than silently preferring one source.
