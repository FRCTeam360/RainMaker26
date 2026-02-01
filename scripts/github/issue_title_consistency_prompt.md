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

- **Hardware Integration**: "Implement [Component] Subsystem" (not "Create" or "Add")
- **Software Features**: "Implement [Feature]" or "Add [Feature]"
- **Testing Tasks**: "Test [What] [Where/How]" (e.g., "Test Flywheel VelocityTorqueCurrentFOC")
- **Setup Tasks**: "Set Up [What]" (not "Setup" - use two words)
- **Research Tasks**: "Research [Topic]" or "Investigate [Issue]"
- **Documentation**: "Update [Document]" or "Document [What]"

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

- **Target**: 3-7 words
- **Maximum**: 10 words
- If too long, use description for details and keep title high-level

## Red Flags to Fix

❌ **Inconsistent verbs**: Some issues say "Implement", others "Create", others "Add" for same type of work
❌ **Vague titles**: "Test Superstructure" → "Test Superstructure Subsystem in Sim"
❌ **Missing context**: "Implement Turret Sim" is good, but "Turret Sim" would be bad
❌ **Inconsistent naming**: "Intake Pivot" vs "IntakePivot" in different titles
❌ **Wrong epic/verb pairing**: Testing epic with "Implement" verb

## Examples

### Before & After

| Before | After | Reason |
|--------|-------|--------|
| "Turret Subsystem" | "Implement Turret Subsystem" | Missing action verb |
| "Create Turret Sim" | "Implement Turret Sim" | Consistency - use "Implement" for subsystem features |
| "Setup unit testing" | "Set Up Unit Testing" | Grammar - two words, proper capitalization |
| "Test flywheel" | "Test and Tune Flywheel VelocityTorqueCurrentFOC" | Specificity - add details from description |
| "Implement simple AprilTag targeting" | "Implement AprilTag Targeting" | Brevity - "simple" is implied, details in description |
| "Add Intake Pivot to Autos" | "Add IntakePivot to Bulletproof Autos" | Consistency - match code naming |

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
- [ ] Titles are 3-7 words (maximum 10)
- [ ] Epic categories align with title structure
- [ ] Titles accurately reflect enhanced descriptions
- [ ] No redundant words ("simple", "basic", "easy")
