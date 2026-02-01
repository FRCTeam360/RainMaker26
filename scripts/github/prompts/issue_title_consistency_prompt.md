# Issue Title Consistency Enhancement Prompt

Standardize and improve the titles of GitHub issues for an FRC robot programming project to ensure consistency, clarity, and scannability. This prompt should be run **after** using the description enhancement prompt.

## Context

You have a set of GitHub issues with enhanced descriptions (created using `issue_description_prompt.md`). Now standardize the titles to match the descriptions and maintain consistency across all issues.

## Objectives

1. **Consistency**: Use consistent action verbs and naming conventions across similar issue types
2. **Clarity**: Titles should clearly indicate what the issue is about without reading the description
3. **Scannability**: Team members should be able to quickly understand the issue from the title alone
4. **Accuracy**: Titles must accurately reflect the enhanced description content

## Standardization Rules

### Action Verb Usage by Epic

Use these standard action verbs consistently:

- **Subsystems**: "Implement [Subsystem Name] Subsystem"
- **Commands**: "Create [Command Name] Command" or "Implement [Feature] Command"
- **Automation**: "Implement [Automation Feature]" or "Add [Automation Feature]"
- **Simulations**: "Implement [Subsystem] Sim" or "Test [Feature] in Sim"
- **Testing**: "Test [Feature/Subsystem]" or "Tune [System Parameter]"
- **Vision**: "Implement [Vision Feature]" or "Calibrate [Vision Hardware]"
- **Autos**: "Implement [Auto Feature]" or "Create [Auto Routine]"
- **Tooling**: "Set Up [Tool/Utility]" or "Port [Utility] from RainMaker25"
- **Architecture**: "Set Up [Architecture Pattern]" or "Refactor to [Pattern]"
- **Administration**: "Clean Up [Area]" or "Update [Documentation]"

### Format Patterns

- **Hardware Integration**: "Implement [Component] Subsystem for [Purpose]" (e.g., "Implement Turret Subsystem for Hub Tracking")
- **Software Features**: "Implement [Feature] [Key Detail]" (e.g., "Implement Snap to Hub Auto-Alignment")
- **Testing Tasks**: "Test [What] [Specific Aspect]" (e.g., "Test Vision Localization Accuracy on Field")
- **Setup Tasks**: "Set Up [What] for [Scope]" (e.g., "Set Up Unit Testing for Subsystems")
- **Research Tasks**: "Research [Topic] for [Goal]" (e.g., "Research Steer Override for Autos")
- **Documentation**: "Update [Document] for [Robot/Component]" (e.g., "Update Motor Controller Spreadsheet for PracticeBot")
- **Command Creation**: "Create [Action] Command [Specific Detail]" (e.g., "Create IntakePivot Stow/Deploy Command")

### Naming Conventions

- **Subsystem Names**: Use PascalCase without spaces (e.g., "IntakePivot" not "Intake Pivot")
- **Technical Terms**: Keep technical terms consistent
  - "AprilTag" (not "April Tag" or "apriltag")
  - "PathPlanner" (not "Path Planner" or "pathplanner")
  - "Limelight" (not "limelight")
  - "SysID" (not "Sys ID" or "sysid")
  - "FMS" (not "fms")
- **Robot Names**: Capitalize properly ("WoodBot" not "woodbot", "PracticeBot" not "practice bot")
- **Abbreviations**: Use consistently (e.g., "IO" for Input/Output, "PID" for Proportional-Integral-Derivative)

### Title Length

- **Target**: 6-9 words (include clarifying detail)
- **Minimum**: 4 words
- **Maximum**: 12 words
- Include one clarifying phrase that specifies purpose, scope, or key feature
- If longer than 12 words, move implementation details to description

### What Details to Include in Titles

**✅ ADD these clarifying details:**
- **Purpose/Goal**: "for Hub Tracking", "for Auto-Alignment", "for Competition"
- **Scope**: "with IO Layers", "for All Subsystems", "Across Workspace"
- **Key Feature**: "Stow/Deploy", "Vision-Based", "Dynamic Selection"
- **What it Affects**: "Shooter Sequence", "Drivetrain Characterization"
- **Specific Component**: "from Hub", "to Arbitrary Point", "on Field"

**❌ DON'T add these (keep in description):**
- Implementation details: "using DCMotorSim", "with PIDController"
- Technology specifics: "using AdvantageKit IO pattern", "following FRC 6328"
- Step-by-step instructions: "Create TurretIO, TurretIOSim, and TurretIOWB"
- Code-level details: "in Constants.WoodBotConstants", "with VelocityTorqueCurrentFOC"

## Red Flags to Fix

❌ **Inconsistent verbs**: Some issues say "Implement", others "Create", others "Add" for same type of work
❌ **Vague titles without purpose**: "Implement Turret Subsystem" → "Implement Turret Subsystem for Hub Tracking"
❌ **Too short, unclear scope**: "Create IntakePivot Command" → "Create IntakePivot Stow/Deploy Command"
❌ **Missing context**: "Test Localization" → "Test Vision Localization Accuracy on Field"
❌ **Inconsistent naming**: "Intake Pivot" vs "IntakePivot" in different titles
❌ **Wrong epic/verb pairing**: Testing epic with "Implement" verb
❌ **Too long with implementation details**: "Implement Turret using DCMotorSim with Mechanism2d Visualization" → move details to description

## Examples

### Before & After

| Before | After | Reason |
|--------|-------|--------|
| "Turret Subsystem" | "Implement Turret Subsystem for Hub Tracking" | Missing action verb + purpose |
| "Implement Turret Subsystem" | "Implement Turret Subsystem for Hub Tracking" | Add purpose for clarity |
| "Create Turret Sim" | "Implement Turret Subsystem Simulation" | Consistency + clearer scope |
| "Setup unit testing" | "Set Up Unit Testing for Subsystems" | Grammar + scope |
| "Test flywheel" | "Test and Tune Flywheel Velocity Control" | Add specific aspect being tested |
| "Implement simple AprilTag targeting" | "Implement AprilTag Auto-Targeting" | Remove "simple", add "Auto-" for clarity |
| "Add Intake Pivot to Autos" | "Add IntakePivot and Turret to Bulletproof Autos" | PascalCase + complete scope |
| "Create IntakePivot Command" | "Create IntakePivot Stow/Deploy Command" | Specify which command feature |
| "Implement Snap to Hub" | "Implement Snap to Hub Auto-Alignment" | Add goal/purpose |
| "Test Localization Accuracy" | "Test Vision Localization Accuracy on Field" | Specify which localization + where |

---

## Input Format

Provide all issues in this format:

```csv
number,title,description,epic
4,"Test Superstructure","Set up the superstructure architecture for the robot.",Architecture
11,"Implement Snap to Hub Using Localization","Automation feature to automatically align robot to the hub.",Automation
...
```

---

## Output Format

Return a markdown table with recommendations:

```markdown
## Title Consistency Analysis

### Issues Requiring Title Updates

| # | Current Title | Recommended Title | Reason | Priority |
|---|---------------|-------------------|--------|----------|
| 4 | Test Superstructure | Set Up Superstructure Architecture | Action verb + specificity | Medium |
| 117 | Implement Turret Subsystem | Implement Turret Subsystem | ✅ No change needed | - |
| ... | ... | ... | ... | ... |

### Summary Statistics
- Total issues: X
- Issues needing updates: Y
- Common issues found:
  - [List common patterns that need fixing]

### Standardization Notes
- [Any project-specific conventions noticed]
- [Recommendations for future issue creation]
```

---

## Usage Instructions

1. Run `issue_description_prompt.md` first to enhance all descriptions
2. Export all issues with their enhanced descriptions to CSV format
3. Copy this prompt and paste the CSV data into the "Input Format" section
4. Feed to LLM to generate title recommendations
5. Review recommendations and apply title updates to GitHub issues
6. Update any related documentation (sprint boards, roadmaps, etc.)

## Quality Checklist

Before finalizing, verify:
- [ ] All similar issue types use consistent action verbs
- [ ] Subsystem names match code exactly (PascalCase)
- [ ] Technical terms are capitalized correctly
- [ ] Titles are 6-9 words (minimum 4, maximum 12)
- [ ] Each title includes clarifying detail (purpose, scope, key feature, or goal)
- [ ] Epic categories align with title structure
- [ ] Titles accurately reflect enhanced descriptions
- [ ] No redundant words ("simple", "basic", "easy")
- [ ] Implementation details are in description, not title
- [ ] Title answers "What specifically?" without explaining "How?"
