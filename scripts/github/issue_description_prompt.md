# Issue Description Enhancement Prompt

Enhance the following GitHub issue description for an FRC (FIRST Robotics Competition) Java robot programming project. The description should be clear and actionable for a Java robot programming student with limited experience.

## Target Audience
High school students learning FRC robot programming with basic Java knowledge but limited robotics experience.

## Required Elements

Each description must include:

1. **Clear Objective**: What needs to be built/tested/implemented in 1-2 sentences
2. **Technical Context**: 
   - Which subsystems/classes are involved
   - What design patterns to follow (e.g., AdvantageKit IO layer pattern)
   - References to existing similar code in the codebase as examples
3. **Implementation Guidance**:
   - Specific file locations or folder structures (e.g., "Create TurretIOSim in subsystems/Turret/")
   - Relevant constants from Constants.java (e.g., CAN IDs, port numbers)
   - WPILib/vendor library APIs to use (e.g., "Use DCMotorSim", "Reference PIDController")
4. **Testing/Verification**: How to validate the work is complete

## Style Guidelines

- **Length**: 2-4 sentences for simple tasks, up to 6-8 sentences for complex tasks
- **Tone**: Instructional but not condescending; assume basic Java knowledge
- **Specificity**: Always reference actual class names, file paths, and constant names from the codebase
- **External References**: Include links to documentation (PathPlanner, CTRE, WPILib) when relevant
- **No Assumptions**: Don't assume students know FRC-specific terminology - briefly explain terms like "IO layer", "SysID", "AprilTag"

## Format Template

```
[ACTION VERB] [WHAT]. [TECHNICAL CONTEXT - which classes/subsystems/patterns]. [IMPLEMENTATION DETAILS - specific files, constants, APIs to use]. [TESTING/VALIDATION - how to verify completion].
```

## Examples

**Bad**: "Implement turret subsystem"

**Good**: "Create a new turret subsystem following the AdvantageKit IO layer pattern (TurretIO interface, TurretIOSim for simulation, TurretIOWB for WoodBot hardware). The turret should rotate to aim at the hub/scoring target. Include methods for setting angle setpoints, reading encoder position, and controlling the motor. Add turret CAN ID to Constants.WoodBotConstants."

---

## Issue to Enhance

**Title**: [ISSUE_TITLE]

**Current Description**: [CURRENT_DESCRIPTION or "empty"]

**Epic/Category**: [EPIC]

**Codebase Context**:
- Robot framework: WPILib 2026, AdvantageKit, CTRE Phoenix 6
- Main subsystems: Drivetrain (swerve), Intake, IntakePivot, Indexer, Flywheel, FlywheelKicker, Hood, Vision
- Project uses IO layer pattern: each subsystem has IO interface + implementations for Sim/WoodBot/PracticeBot
- Constants defined in Constants.java with nested classes (WoodBotConstants, SimulationConstants)
- Commands use factory pattern (CommandFactory.java, may migrate to Superstructure pattern)

**Enhanced Description**:
[OUTPUT HERE]

---

## Usage Instructions

1. Copy this entire prompt
2. Replace `[ISSUE_TITLE]`, `[CURRENT_DESCRIPTION]`, and `[EPIC]` with actual issue data
3. Update "Codebase Context" section if project structure changes
4. Feed to LLM to generate enhanced description
5. Paste result back into GitHub issue body
