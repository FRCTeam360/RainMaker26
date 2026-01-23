# GitHub Issues CSV Import/Export using GitHub CLI

This package includes everything you need to import issues from a spreadsheet into GitHub and export current issues to CSV using the GitHub CLI.

## Files Included

- `github_issues_to_add.csv` - Template CSV file for importing issues
- `import_issues.ps1` - PowerShell script to import issues from CSV
- `export_issues.ps1` - PowerShell script to export current issues to CSV
- `README.md` - This file

## Prerequisites

1. **Install GitHub CLI** (if not already installed):
   - **Windows (Chocolatey)**: `choco install gh`
   - **Windows (WinGet)**: `winget install --id GitHub.cli`
   - **Windows (Manual)**: Download from https://cli.github.com/
   - **macOS**: `brew install gh`
   - **Linux**: See https://github.com/cli/cli#installation

2. **Authenticate with GitHub**:
   ```bash
   gh auth login
   ```
   Follow the prompts to authenticate.

## CSV Format

Your CSV file should have these columns (header row required):

| Column    | Required | Description            | Example                       |
| --------- | -------- | ---------------------- | ----------------------------- |
| title     | Yes      | Issue title            | "Fix login bug"               |
| body      | No       | Issue description      | "Users can't login on mobile" |
| labels    | No       | Comma-separated labels | "bug,high-priority"           |
| assignee  | No       | GitHub username        | "johndoe"                     |
| milestone | No       | Milestone name         | "Sprint 1"                    |
| epic      | No       | Project board Epic     | "Subsystems"                  |

**Important Notes:**

- Fields with commas must be wrapped in quotes
- Labels within a field are separated by commas without spaces: `"bug,feature"`
- Leave fields empty if not needed (but keep the commas)
- Milestones must already exist in your repository
- Epic must match one of: `Architecture`, `Subsystems`, `Automation`, `Autos`, `Tooling`, `Testing/Prototyping`, `Logging`, `Simulations`, `Vision`, `Administration`

## Usage

### Exporting Issues

To export current open issues from your repository to a CSV file:

```powershell
cd C:\path\to\your\repo
.\export_issues.ps1
```

This will create `github_issues_export.csv` with all open issues.

To specify a custom filename or repository:

```powershell
.\export_issues.ps1 my_export.csv owner/repo-name
```

To include epic information from the project board:

```powershell
.\export_issues.ps1 -IncludeEpic
```

**Note:** Epic information requires the script to be configured for the correct project board (currently set to FRCTeam360's 2026 Rebuilt Season board).

### Importing Issues

#### Option 1: From within your repository directory

```powershell
cd C:\path\to\your\repo
.\import_issues.ps1 github_issues_to_add.csv
```

#### Option 2: Specify repository explicitly

```powershell
.\import_issues.ps1 github_issues_to_add.csv owner/repo-name
```

#### Option 3: Use a custom CSV file

```powershell
.\import_issues.ps1 my_custom_issues.csv owner/repo-name
```

## Step-by-Step Instructions

1. **Edit the CSV template** with your issues:
   - Open `github_issues_to_add.csv` in Excel, Google Sheets, or any spreadsheet app
   - Replace the example issues with your own
   - Save as CSV

2. **Make sure you're authenticated**:

   ```bash
   gh auth status
   ```

3. **Run the import script**:

   ```powershell
   .\import_issues.ps1 github_issues_to_add.csv owner/repo-name
   ```

4. **Check your repository** - issues should now appear!

## Adding Issues to a Project Board

The PowerShell script automatically adds created issues to the **2026 Rebuilt Season** project board and sets the **Epic** field if specified. You'll see confirmation in the output:

```
Creating issue: Some Task
[OK] Created successfully
  -> Added to Rebuilt board
  -> Epic set to: Subsystems
```

### Changing the Project Board

To use a different project board, first list available projects:

```bash
gh project list --owner FRCTeam360
```

This outputs something like:

```
NUMBER  NAME                  STATE  ID
2       2026 Rebuilt Season   open   PVT_kwDOAKOwYc4BMevW
1       Preseason Kanban      open   PVT_kwDOAKOwYc4BCH01
```

Then update the project configuration variables at the top of `import_issues.ps1`:

```powershell
$ProjectOwner = "FRCTeam360"
$ProjectNumber = 2
$ProjectId = "PVT_kwDOAKOwYc4BMevW"
```

To update Epic options for a different project, run:

```bash
gh project field-list PROJECT_NUMBER --owner FRCTeam360 --format json
```

### Manual Addition (Alternative)

You can also manually add issues to a project board:

**Using GitHub CLI:**

```bash
gh project item-add PROJECT_NUMBER --owner FRCTeam360 --url ISSUE_URL
```

**Using the GitHub Web Interface:**

1. Go to your repository's "Projects" tab
2. Open your project board
3. Click "+ Add item" and search for your newly created issues
4. Add them to the appropriate columns

## Troubleshooting

**"gh: command not found" or "gh is not recognized"**

- GitHub CLI is not installed. See prerequisites above.
- On Windows, you may need to restart PowerShell after installation.

**"Error: not authorized"**

- Run `gh auth login` to authenticate

**PowerShell Execution Policy Error (Windows)**

- If you get "cannot be loaded because running scripts is disabled":
  ```powershell
  Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
  ```

**"Error: milestone not found"**

- Create the milestone in your repository first, or remove milestone values from CSV

**"Error: label not found"**

- Labels are auto-created, but if you have label restrictions, create them first

**Rate limiting**

- The script includes a 1-second delay between issues
- For large imports (100+ issues), consider breaking into smaller batches

## Tips

- **Test first**: Create a test repository and import a few sample issues to verify
- **Backup**: Keep your CSV file as a backup/record
- **Labels**: Use consistent label names across issues for better organization
- **Assignees**: Make sure usernames exist and have access to the repository
- **Bulk editing**: After import, you can bulk-edit issues in the GitHub web interface

## Example CSV Content

```csv
title,body,labels,assignee,milestone,epic
"Setup CI/CD","Configure GitHub Actions","devops,automation",,Sprint 1,Tooling
"Fix mobile bug","Login fails on iOS","bug,high-priority",johndoe,Sprint 1,Subsystems
"Add docs","Write API documentation","documentation",,Sprint 2,Administration
```

## Need Help?

- GitHub CLI Docs: https://cli.github.com/manual/
- GitHub Issues Docs: https://docs.github.com/en/issues
- GitHub Projects Docs: https://docs.github.com/en/issues/planning-and-tracking-with-projects
