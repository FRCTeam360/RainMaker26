# Add Missing Issues to Project Board Script (PowerShell)
# This script finds issues that exist in the repo but are not in the Rebuilt project board,
# then adds them to the board.
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login

param(
    [Parameter(Position=0)]
    [string]$Repo = "FRCTeam360/RainMaker26",

    [int]$ProjectNumber = 2,

    [string]$ProjectOwner = "FRCTeam360",

    [switch]$DryRun = $false
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

Write-Host "Finding missing issues in the Rebuilt project board..." -ForegroundColor Green
Write-Host ""

if ($DryRun) {
    Write-Host "[DRY RUN MODE - No changes will be made]" -ForegroundColor Cyan
    Write-Host ""
}

# Step 1: Fetch all issues from the repository
Write-Host "Fetching all issues from $Repo..." -ForegroundColor Yellow
$ghArgs = @("issue", "list", "--limit", "1000", "--state", "all", "--json", "number,title,url")
$ghArgs += @("--repo", $Repo)

try {
    $issuesJson = & gh @ghArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error fetching issues: $issuesJson" -ForegroundColor Red
        exit 1
    }
    $repoIssues = $issuesJson | ConvertFrom-Json
} catch {
    Write-Host "Error parsing issues JSON: $_" -ForegroundColor Red
    exit 1
}

Write-Host "Found $($repoIssues.Count) issues in repo" -ForegroundColor Cyan

# Step 2: Fetch all items from the Rebuilt project board
Write-Host "Fetching items from Rebuilt project board (Project #$ProjectNumber)..." -ForegroundColor Yellow
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

# Extract issue numbers from project items
$boardIssueNumbers = @()
foreach ($item in $projectItems.items) {
    if ($item.content.type -eq "Issue") {
        $boardIssueNumbers += $item.content.number.ToString()
    }
}

Write-Host "Found $($boardIssueNumbers.Count) issues in Rebuilt board" -ForegroundColor Cyan
Write-Host ""

# Step 3: Find missing issues
$missingIssues = $repoIssues | Where-Object { $_.number.ToString() -notin $boardIssueNumbers }

if ($missingIssues.Count -eq 0) {
    Write-Host "All issues are already in the Rebuilt board!" -ForegroundColor Green
    exit 0
}

Write-Host "Found $($missingIssues.Count) missing issues:" -ForegroundColor Yellow
foreach ($issue in $missingIssues) {
    Write-Host "  #$($issue.number): $($issue.title)" -ForegroundColor White
}
Write-Host ""

# Step 4: Add missing issues to the board
$addedCount = 0
$failedCount = 0

foreach ($issue in $missingIssues) {
    Write-Host "Adding #$($issue.number) to board: $($issue.title)" -ForegroundColor Yellow

    if ($DryRun) {
        Write-Host "  -> [DRY RUN] Would add using URL: $($issue.url)" -ForegroundColor Cyan
        $addedCount++
        continue
    }

    try {
        $result = gh project item-add $ProjectNumber --owner $ProjectOwner --url $issue.url 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  -> Added successfully" -ForegroundColor Green
            $addedCount++
        } else {
            Write-Host "  -> Failed: $result" -ForegroundColor Red
            $failedCount++
        }
    } catch {
        Write-Host "  -> Failed: $_" -ForegroundColor Red
        $failedCount++
    }

    # Small delay to avoid rate limiting
    Start-Sleep -Milliseconds 500
}

Write-Host ""
Write-Host "Complete!" -ForegroundColor Green
Write-Host "Added: $addedCount issues"
if ($failedCount -gt 0) {
    Write-Host "Failed: $failedCount issues" -ForegroundColor Red
}
