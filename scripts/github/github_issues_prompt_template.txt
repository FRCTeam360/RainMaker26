# GitHub Issues CSV Generator Prompt

Use this prompt when adding new tasks to our GitHub issues tracker.

---

## Context

I'm on an FRC robotics team. I need you to convert my planned tasks into a GitHub issues CSV file. I've also included a sheet with our current tasks so we don't make redundant tasks. If you're missing context, please ask clarifying questions.

## CSV Format

Output a CSV with these columns, with ALL fields quoted to escape commas:
- title
- body
- labels
- assignee
- milestone
- epic
- priority

## Available Labels

- `bug` - Something isn't working
- `documentation` - Improvements or additions to documentation
- `duplicate` - This issue or pull request already exists
- `enhancement` - New feature or request
- `good first issue` - Good for newcomers
- `help wanted` - Extra attention is needed
- `invalid` - This doesn't seem right
- `question` - Further information is requested
- `wontfix` - This will not be worked on

## Milestones (Build Season Weeks)

- Week 1: 1/10/2026 - 1/17/2026
- Week 2: 1/18/2026 - 1/24/2026
- Week 3: 1/25/2026 - 1/31/2026
- Week 4: 2/1/2026 - 2/7/2026
- Week 5: 2/8/2026 - 2/14/2026
- Week 6: 2/15/2026 - 2/21/2026
(Add more weeks as needed)

## Epics

- Architecture
- Subsystems
- Automation
- Autos
- Tooling
- Testing/Prototyping
- Logging
- Simulations
- Vision
- Administration
- Commands

## Priority (optional)

- Number One Highest Priority
- High Priority
- Low Priority

Leave blank for normal priority items.

## Formatting Rules

1. **Rephrase questions as action items** (e.g., "What are the flywheel motors?" → "Determine flywheel motor type")
2. **Use semicolons instead of commas** in body text to avoid CSV parsing issues
3. **Add descriptions** where context is clear; leave blank otherwise
4. **Mark emphasized items** (originally in __underscores__) with "High Priority" in the priority column
5. **Assign labels** based on task type:
   - `enhancement` for new features/implementations
   - `question` for investigation items or things needing answers from others
   - `documentation` for setup/tracking tasks
6. **Leave assignee blank** - we assign during sprint planning
7. **Output a companion markdown file** listing any rephrased tasks with their original wording

## Current Week

[UPDATE THIS: e.g., "We are currently in Week 3"]

---

## My Tasks

[PASTE YOUR TASKS HERE]

---

Please convert these tasks into a properly formatted CSV file and provide a companion doc with original phrasing for any rephrased items.
