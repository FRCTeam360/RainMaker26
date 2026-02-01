# Issue Deduplication and Consolidation Prompt

Identify duplicate, overlapping, or redundant GitHub issues in an FRC robot programming project. This prompt should be run **after** using both the description enhancement and title consistency prompts to ensure issues are properly formatted before analysis.

## Context

You have a set of GitHub issues with:
- Enhanced descriptions (from `issue_description_prompt.md`) with clear objectives, implementation guidance, and acceptance criteria
- Standardized titles (from `issue_title_consistency_prompt.md`) with consistent verbs and clarifying details

Now identify issues that are duplicates, have significant overlap, or should be consolidated.

## Objectives

1. **Identify Exact Duplicates**: Same work described in different issues
2. **Find Semantic Duplicates**: Different wording but same implementation
3. **Detect Overlap**: Issues with partial overlap that could be merged
4. **Spot Dependencies**: Issues that are really sub-tasks of another issue
5. **Flag Overly Broad Issues**: Issues that should be split into multiple smaller issues

## Types of Duplication to Detect

### 1. Exact Duplicates
Same subsystem/feature, same implementation approach, same acceptance criteria.

**Example:**
- Issue #117: "Implement Turret Subsystem for Hub Tracking"
- Issue #169: "Implement Turret Auto-Tracking"
→ Same work if #169 is the tracking feature of #117

### 2. Semantic Duplicates
Different titles/descriptions but implementing the same functionality.

**Example:**
- Issue #37: "Implement AprilTag Targeting"
- Issue #77: "Implement Localization Pose from Hub"
→ Could be duplicate if both use AprilTags for hub pose

### 3. Partial Overlap
Issues share some implementation work but have different scopes.

**Example:**
- Issue #165: "Create Automated Shooting Command"
- Issue #189: "Create ShotCalculator Class"
→ #189 is likely a dependency of #165

### 4. Scope Confusion
One issue is a subset or superset of another.

**Example:**
- Issue #159: "Implement Full Robot Simulation"
- Issue #142: "Implement Turret Subsystem Simulation"
→ #142 is part of #159's scope

### 5. Implementation vs Testing
Separate issues for implementation and testing of the same feature.

**Example:**
- Issue #117: "Implement Turret Subsystem"
- Issue #142: "Implement Turret Sim"
- Issue #101: "Test Turret in Sim"
→ Consider if testing should be part of implementation issue

## Analysis Criteria

When comparing two issues, check:

### Title Similarity
- ✅ Same subsystem/component name (e.g., "Turret", "IntakePivot")
- ✅ Same action on same target (e.g., "Implement Shooting", "Create Shooting")
- ✅ Similar purpose/goal (e.g., "for Hub Tracking" vs "to Track Hub")

### Description Overlap
- ✅ Same file paths mentioned (e.g., both create TurretIOSim)
- ✅ Same constants referenced (e.g., both use TURRET_CAN_ID)
- ✅ Same subsystems involved
- ✅ Same acceptance criteria or testing methods

### Epic/Category Relationship
- ✅ Same epic but different development phases (Implement → Sim → Test)
- ✅ Different epics but same underlying work (Subsystems + Simulations)

### Dependency Indicators
Look for phrases in descriptions:
- "requires", "depends on", "after", "once X is complete"
- "uses [other feature]", "integrates with [other subsystem]"
- References to other issue numbers

## Decision Framework

### When to Mark as DUPLICATE (Merge/Close)
- ❌ Exact same work in both issues
- ❌ One issue is completely covered by another
- ❌ Same acceptance criteria and deliverables
- ❌ No additional value in keeping separate

**Action**: Close duplicate, reference kept issue in comment

### When to Mark as DEPENDENT (Link)
- 🔗 One issue must be completed before the other
- 🔗 One issue is a prerequisite or sub-task
- 🔗 One issue creates infrastructure used by the other

**Action**: Add "blocked by" relationship in GitHub, keep both issues

### When to Mark as OVERLAPPING (Consolidate)
- 🔄 Significant shared implementation work (>50%)
- 🔄 Would be confusing to work on separately
- 🔄 Acceptance criteria overlap substantially

**Action**: Merge into one issue, expand scope, close redundant issue

### When to Mark as SPLIT NEEDED (Too Broad)
- 📊 Issue covers multiple subsystems or major features
- 📊 Acceptance criteria list >5 distinct items
- 📊 Implementation guidance spans multiple unrelated files
- 📊 Could reasonably be 3+ separate issues

**Action**: Split into multiple focused issues, close original

### When to Keep SEPARATE
- ✅ Different phases of work (Implement → Test can be separate)
- ✅ Different robots/contexts (WoodBot vs PracticeBot)
- ✅ Different team members working in parallel
- ✅ Minimal overlap (<25% shared work)

**Action**: No change, possibly add cross-references

## Output Format

Return a detailed analysis:

```markdown
## Issue Deduplication Analysis

### Exact Duplicates (Merge/Close)

| Primary Issue | Duplicate Issue(s) | Reason | Recommended Action |
|---------------|-------------------|--------|-------------------|
| #117: Implement Turret Subsystem for Hub Tracking | #169: Implement Turret Auto-Tracking | Auto-tracking is core feature of turret subsystem | Close #169, reference #117 |
| ... | ... | ... | ... |

### Dependencies (Link/Block)

| Blocking Issue | Blocked Issue | Dependency Reason | Recommended Action |
|----------------|---------------|-------------------|-------------------|
| #117: Implement Turret Subsystem | #142: Implement Turret Sim | Sim requires base subsystem structure | Add "blocked by #117" to #142 |
| ... | ... | ... | ... |

### Overlapping Issues (Consolidate)

| Issue 1 | Issue 2 | Overlap Description | Recommended Action |
|---------|---------|---------------------|-------------------|
| #165: Create Automated Shooting Command | #189: Create ShotCalculator Class | ShotCalculator is core component of shooting command | Merge into #165, expand description |
| ... | ... | ... | ... |

### Issues Too Broad (Split)

| Issue | Why Too Broad | Suggested Split |
|-------|---------------|----------------|
| #159: Implement Full Robot Simulation | Covers 7+ subsystems, each needs separate sim implementation | Split into per-subsystem sim issues (#142, #133, etc.) |
| ... | ... | ... |

### Related But Separate (Cross-Reference)

| Issue 1 | Issue 2 | Relationship | Recommended Action |
|---------|---------|--------------|-------------------|
| #37: Implement AprilTag Targeting | #72: Calibrate Limelight Cameras | Targeting uses calibrated cameras | Add cross-reference comment |
| ... | ... | ... | ... |

### Summary Statistics
- Total issues analyzed: X
- Exact duplicates found: Y
- Dependencies identified: Z
- Overlapping issues: A
- Issues to split: B
- Related issues to cross-reference: C

### Consolidation Recommendations
1. **High Priority**: [List issues that should definitely be merged/closed]
2. **Medium Priority**: [List issues that could be consolidated for efficiency]
3. **Low Priority**: [List nice-to-have consolidations]

### Project Structure Notes
- [Any patterns in duplication - e.g., "Implementation and testing often separate"]
- [Recommendations for preventing future duplicates]
- [Suggested epic/label structure changes]
```

---

## Input Format

Provide all issues with enhanced descriptions and standardized titles:

```csv
number,title,body,epic
4,"Set Up Superstructure Architecture","Set up the superstructure architecture...",Architecture
11,"Implement Snap to Hub","Automation feature to automatically align...",Automation
117,"Implement Turret Subsystem for Hub Tracking","Create a new turret subsystem following...",Subsystems
...
```

---

## Analysis Process

### Step 1: Group by Subsystem/Component
Group issues mentioning the same subsystems (Turret, IntakePivot, Flywheel, etc.)

### Step 2: Group by Epic Progression
Look for Implementation → Simulation → Testing chains

### Step 3: Extract Key Entities
From each description, extract:
- File paths mentioned
- Class names created/modified
- Constants referenced
- Subsystems involved
- APIs used

### Step 4: Compare Entities
Issues with >70% entity overlap are likely duplicates

### Step 5: Check Dependencies
Look for explicit dependency language in descriptions

### Step 6: Validate with Context
Consider robot development workflow:
- Is it normal to have separate Implement/Sim/Test issues?
- Are these phases or duplicates?

---

## Usage Instructions

1. Run `issue_description_prompt.md` first to enhance all descriptions
2. Run `issue_title_consistency_prompt.md` to standardize all titles
3. Export all issues with enhanced descriptions and titles to CSV format
4. Copy this prompt and paste the CSV data into the "Input Format" section
5. Feed to LLM to generate deduplication analysis
6. Review recommendations with team before merging/closing issues
7. Update GitHub issues based on approved recommendations
8. Add cross-references and "blocked by" relationships where identified

## Quality Checklist

Before finalizing deduplication recommendations:
- [ ] Verified exact duplicates have 100% overlapping scope
- [ ] Checked that "dependencies" are truly blocking relationships
- [ ] Confirmed overlapping issues would benefit from consolidation
- [ ] Validated that broad issues actually need splitting (not premature)
- [ ] Ensured no valuable information is lost in consolidation
- [ ] Considered team workflow (parallel work, phases, etc.)
- [ ] Double-checked issue numbers and references are correct

## Special Considerations for FRC Projects

### Normal Patterns (NOT duplicates)
- **Subsystem + Sim + Test**: Separate issues for implementation phases
- **WoodBot + PracticeBot**: Same feature for different robot hardware
- **Research + Implement**: Spike followed by implementation
- **Command + Subsystem**: Command using a subsystem (not duplicate)

### Likely Duplicates
- Two "Implement X Subsystem" with same robot target
- Two simulation issues for same subsystem
- Multiple "Test X" without different test contexts
- Same feature with slightly different wording

### Dependencies to Watch For
- ShotCalculator → Automated Shooting Command
- Vision Localization → Snap to Hub
- Subsystem Implementation → Subsystem Simulation
- IO Layer Setup → Hardware Testing
