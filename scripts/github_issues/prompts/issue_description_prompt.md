# Issue Description Enhancement Prompt

Enhance the following GitHub issue description for an FRC (FIRST Robotics Competition) Java robot programming project. The description should be clear and actionable for a Java robot programming student with limited experience.

## Target Audience
High school students learning FRC robot programming with basic Java knowledge but limited robotics experience.

## Required Elements

Each description must include:

1. **Clear Objective**: What needs to be built/tested/implemented in 1-2 sentences
   - Must align with and expand upon the issue title
   - Answer "What specifically?" in more detail than the title
   
2. **Technical Context**: 
   - Which subsystems/classes are involved
   - What design patterns to follow (e.g., AdvantageKit IO layer pattern)
   - References to existing similar code in the codebase as examples
   - Dependencies on other issues or subsystems
   
3. **Implementation Guidance**:
   - Specific file locations or folder structures (e.g., "Create TurretIOSim in subsystems/Turret/")
   - Relevant constants from Constants.java (e.g., CAN IDs, port numbers)
   - WPILib/vendor library APIs to use (e.g., "Use DCMotorSim", "Reference PIDController")
   - Step-by-step approach or key implementation points
   
4. **Acceptance Criteria**: Clear, testable conditions for completion
   - "Verify that [specific behavior] works as expected"
   - "Test using [tool/method] and confirm [specific outcome]"
   - "Ensure [metric/measurement] meets [target value]"
   
5. **Testing/Verification**: How to validate the work is complete
   - Specific simulation scenarios to test
   - Real hardware tests to perform
   - Expected outputs or behaviors
   - Tools to use for validation (AdvantageScope, Phoenix Tuner, etc.)

## Style Guidelines

- **Length**: 3-5 sentences for simple tasks, 6-10 sentences for complex tasks
- **Tone**: Instructional but not condescending; assume basic Java knowledge
- **Specificity**: Always reference actual class names, file paths, and constant names from the codebase
- **External References**: Include links to documentation (PathPlanner, CTRE, WPILib) when relevant
- **No Assumptions**: Don't assume students know FRC-specific terminology - briefly explain terms like "IO layer", "SysID", "AprilTag"
- **Actionable**: Student should be able to start work immediately after reading
- **Complete**: Description should provide enough detail that the title's purpose/scope makes sense
- **Testable**: Include specific success criteria that can be verified

## Actionability Requirements

A description is actionable when:
- ✅ Student knows exactly what files to create/modify
- ✅ Student knows which classes/methods to reference as examples
- ✅ Student knows what constants/configuration values to use
- ✅ Student knows how to test their implementation
- ✅ Student knows what "done" looks like with specific criteria

A description needs improvement if:
- ❌ Only repeats the title in different words
- ❌ Missing file locations or class names
- ❌ No testing/verification steps
- ❌ Vague phrases like "implement properly" or "make it work"
- ❌ No acceptance criteria or success conditions

## Format Template

```
[CLEAR OBJECTIVE - expand on title's purpose/scope]. [TECHNICAL CONTEXT - which classes/subsystems/patterns, references to similar code]. [IMPLEMENTATION DETAILS - specific files, folder structure, constants from Constants.java, APIs to use, step-by-step approach]. [ACCEPTANCE CRITERIA - specific testable conditions]. [TESTING/VALIDATION - how to verify, tools to use, expected outputs].
```

## Examples

### Example 1: Subsystem Implementation

**Title**: "Implement Turret Subsystem for Hub Tracking"

**Bad Description**: "Implement turret subsystem"

**Good Description**: "Create a new turret subsystem following the AdvantageKit IO layer pattern (TurretIO interface, TurretIOSim for simulation, TurretIOWB for WoodBot hardware). The turret should rotate to aim at the hub/scoring target using a motor with encoder feedback. Include methods for setting angle setpoints, reading current encoder position, and controlling the motor. Add the turret CAN ID to Constants.WoodBotConstants. Reference the Hood subsystem (subsystems/Hood/) as an example of the IO layer structure and motor control patterns. Success criteria: Turret can rotate to specified angles in simulation with visualization on Mechanism2d, and motor controller configuration is verified in Phoenix Tuner."

### Example 2: Command Creation

**Title**: "Create IntakePivot Stow/Deploy Command"

**Bad Description**: "Create a command that controls the IntakePivot subsystem"

**Good Description**: "Create a command that controls the IntakePivot subsystem to move between stowed (0°) and deployed (90°) positions. Use the IntakePivotIOSim implementation for initial testing in simulation. The command should accept a target angle parameter and use the pivot's built-in feedback control (PID or motion profiling) to smoothly reach the setpoint. Add the command to CommandFactory.java following the existing pattern (see basicIntakeCmd() as example). Include IntakePivotVisualizer to show the pivot angle on the simulation dashboard. Acceptance criteria: Command completes when pivot reaches target angle within ±2 degrees, visualizer updates in real-time, and no motor stalls or oscillation occurs. Test by running simulation and calling the command with different target angles."

### Example 3: Testing Task

**Title**: "Test Vision Localization Accuracy on Field"

**Bad Description**: "Validate localization code accuracy with real hardware on the field"

**Good Description**: "Validate the Vision subsystem's localization accuracy using real hardware on a full-size field with properly positioned AprilTags. Place the robot at known positions (corners, center, amp zone) and compare the Vision-reported pose from VisionIOLimelight against the known ground truth positions. Use the Limelight web interface to verify AprilTag detection and AdvantageScope to log pose data. Test both stationary positioning and moving scenarios (driving straight, rotating in place). Acceptance criteria: Average position error < 5cm, average rotation error < 3 degrees across 10 test positions, no pose jumps > 10cm when tags are in view, and pose updates at > 10Hz. Document any problematic zones where tag visibility is poor."

---

## Coordination with Issue Titles

Descriptions must expand on and support the title's purpose/scope:

**Title provides**: Action verb + What + Clarifying detail (purpose/scope/goal)
**Description provides**: How + Where + With what + Success criteria

### Title-Description Alignment

| Title Element | Description Must Include |
|---------------|-------------------------|
| "Implement Turret Subsystem" | Which files to create, what pattern to follow, what methods to include |
| "for Hub Tracking" | How tracking works, what inputs/sensors, what the control loop does |
| "Create IntakePivot Command" | Where in codebase, which factory/container, what the command does |
| "Stow/Deploy" | Specific angles, speed/motion profile, how to verify positions |
| "Test Vision Localization" | What to measure, what tools to use, what values indicate success |
| "on Field" | Field setup requirements, test positions, real-world conditions |

### Red Flags for Descriptions

❌ Description just repeats title in sentence form
❌ Title mentions specific feature, description is generic
❌ Title promises scope ("for All Subsystems"), description only covers one
❌ No explanation of title's purpose/goal
❌ Student would need to ask "But where do I start?" or "How do I test this?"

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
6. **After all descriptions are enhanced**, run `issue_title_consistency_prompt.md` to ensure titles align with enhanced descriptions

## Quality Checklist

Before finalizing each description, verify:
- [ ] Description expands on the title's purpose/scope/goal
- [ ] Specific file paths and class names are included
- [ ] Constants from Constants.java are referenced where applicable
- [ ] Similar existing code is referenced as an example
- [ ] Clear acceptance criteria are defined
- [ ] Testing steps with specific tools are provided
- [ ] Student knows exactly what "done" looks like
- [ ] No FRC jargon without brief explanation
- [ ] Length is appropriate (3-5 sentences for simple, 6-10 for complex)
