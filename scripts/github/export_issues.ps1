# GitHub Issues CSV Export Script (PowerShell)
# This script uses GitHub CLI (gh) to export issues from a repository to a CSV file, optionally filtered by milestone
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login
# 3. Navigate to your repository directory or set $Repo variable below

param(
    [Parameter(Position=0)]
    [string]$CsvFile = "github_issues_export.csv",

    [Parameter(Position=1)]
    [string]$Repo = "FRCTeam360/RainMaker26",

    [string]$Milestone = "",

    [int]$ProjectNumber = 2,

    [string]$ProjectOwner = "FRCTeam360"
)

# Check if gh CLI is installed
if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
    Write-Host "Error: GitHub CLI (gh) is not installed." -ForegroundColor Red
    Write-Host "Install it with: choco install gh" -ForegroundColor Yellow
    Write-Host "Or download from: https://cli.github.com/" -ForegroundColor Yellow
    exit 1
}

# Check if authenticated
$authStatus = gh auth status 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Not authenticated with GitHub CLI." -ForegroundColor Red
    Write-Host "Run: gh auth login" -ForegroundColor Yellow
    exit 1
}

Write-Host "Starting GitHub Issues export to: $CsvFile" -ForegroundColor Green
Write-Host ""

# Build gh issue list command
$ghArgs = @("issue", "list", "--limit", "1000", "--state", "all", "--json", "number,title,body,labels,assignees,milestone,url")

# Add repository if specified
if (-not [string]::IsNullOrWhiteSpace($Repo)) {
    $ghArgs += @("--repo", $Repo)
}

# Add milestone filter if specified
if (-not [string]::IsNullOrWhiteSpace($Milestone)) {
    $ghArgs += @("--milestone", $Milestone)
}

# Execute command to get issues
try {
    $issuesJson = & gh @ghArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error fetching issues: $issuesJson" -ForegroundColor Red
        exit 1
    }

    $issues = $issuesJson | ConvertFrom-Json
} catch {
    Write-Host "Error parsing issues JSON: $_" -ForegroundColor Red
    exit 1
}

Write-Host "Found $($issues.Count) issues" -ForegroundColor Cyan

# Fetch project items from the FRCTeam360 Rebuilt board
try {
    $projectItemsJson = gh project item-list $ProjectNumber --owner $ProjectOwner --format json --limit 500 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error fetching project items: $projectItemsJson" -ForegroundColor Red
        exit 1
    }

    $projectItems = $projectItemsJson | ConvertFrom-Json
} catch {
    Write-Host "Error parsing project items JSON: $_" -ForegroundColor Red
    exit 1
}

# Extract issue numbers and URLs from project items
$issueNumbers = @()
$epicMap = @{}
foreach ($item in $projectItems.items) {
    if ($item.content.type -eq "Issue") {
        # Get issue number directly from content
        $issueNumber = $item.content.number.ToString()
        $issueNumbers += $issueNumber

        # Get epic field value (directly available in the item)
        if ($item.epic) {
            $epicMap[$issueNumber] = $item.epic
        }
    } else {
      # print out the issue number of missing issues
      Write-Host "Warning: Issue not found in Rebuilt board: $($item.content.number.ToString())" -ForegroundColor Yellow
    }
}

Write-Host "Found $($issueNumbers.Count) issues in the Rebuilt board" -ForegroundColor Cyan

# Find and output missing issues (in repo but not in Rebuilt board)
$missingIssues = $issues | Where-Object { $_.number.ToString() -notin $issueNumbers }
Write-Host "Missing issues (not in Rebuilt board):" -ForegroundColor Yellow
$missingIssues | ForEach-Object { Write-Host "Issue #$($_.number): $($_.title)" -ForegroundColor Yellow }

# Fetch full issue details for each issue number
$issues = @()
foreach ($issueNumber in $issueNumbers) {
    Write-Host "Fetching details for issue #$issueNumber..." -ForegroundColor Yellow

    $ghArgs = @("issue", "view", $issueNumber, "--json", "number,title,body,labels,assignees,milestone,url")

    # Add repository if specified
    if (-not [string]::IsNullOrWhiteSpace($Repo)) {
        $ghArgs += @("--repo", $Repo)
    }

    try {
        $issueJson = & gh @ghArgs 2>&1
        if ($LASTEXITCODE -eq 0) {
            $issue = $issueJson | ConvertFrom-Json
            $issues += $issue
        } else {
            Write-Host "Warning: Could not fetch issue #$issueNumber : $issueJson" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "Warning: Error fetching issue #$issueNumber : $_" -ForegroundColor Yellow
    }
}


# Convert issues to CSV format
$csvData = @()

foreach ($issue in $issues) {
    # Format labels as comma-separated string
    $labelsStr = ""
    if ($issue.labels -and $issue.labels.Count -gt 0) {
        $labelsStr = ($issue.labels | ForEach-Object { $_.name }) -join ","
    }

    # Format assignees (take first one if multiple)
    $assigneeStr = ""
    if ($issue.assignees -and $issue.assignees.Count -gt 0) {
        $assigneeStr = $issue.assignees[0].login
    }

    # Get milestone title
    $milestoneStr = ""
    if ($issue.milestone) {
        $milestoneStr = $issue.milestone.title
    }

    # Get epic from map
    $epicStr = ""
    if ($epicMap.ContainsKey($issue.number.ToString())) {
        $epicStr = $epicMap[$issue.number.ToString()]
    }

    # Create CSV object
    $csvObject = [PSCustomObject]@{
        title     = $issue.title
        body      = $issue.body -replace "`r`n", " " -replace "`n", " "  # Replace newlines with spaces
        labels    = $labelsStr
        assignee  = $assigneeStr
        milestone = $milestoneStr
        epic      = $epicStr
    }

    $csvData += $csvObject
}

# Export to CSV
try {
    $csvData | Export-Csv -Path $CsvFile -NoTypeInformation -Encoding UTF8
    Write-Host "Successfully exported $($csvData.Count) issues to: $CsvFile" -ForegroundColor Green
} catch {
    Write-Host "Error exporting to CSV: $_" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Export complete!" -ForegroundColor Green
