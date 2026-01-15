# GitHub Issues CSV Import Script (PowerShell)
# This script uses GitHub CLI (gh) to create issues from a CSV file
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login
# 3. Navigate to your repository directory or set $Repo variable below

param(
    [Parameter(Position=0)]
    [string]$CsvFile = "github_issues_template.csv",
    
    [Parameter(Position=1)]
    [string]$Repo = ""
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

# Check if CSV file exists
if (-not (Test-Path $CsvFile)) {
    Write-Host "Error: CSV file '$CsvFile' not found." -ForegroundColor Red
    Write-Host "Usage: .\import_issues.ps1 [csv_file] [owner/repo]" -ForegroundColor Yellow
    exit 1
}

Write-Host "Starting GitHub Issues import from: $CsvFile" -ForegroundColor Green
Write-Host ""

# Read CSV file
$issues = Import-Csv -Path $CsvFile

$issuesCreated = 0
$issuesFailed = 0

foreach ($issue in $issues) {
    # Skip empty lines
    if ([string]::IsNullOrWhiteSpace($issue.title)) {
        continue
    }
    
    Write-Host "Creating issue: $($issue.title)" -ForegroundColor Yellow
    
    # Build gh issue create command arguments
    $args = @("issue", "create", "--title", $issue.title)
    
    # Add repository if specified
    if (-not [string]::IsNullOrWhiteSpace($Repo)) {
        $args += "--repo"
        $args += $Repo
    }
    
    # Add body if present
    if (-not [string]::IsNullOrWhiteSpace($issue.body)) {
        $args += "--body"
        $args += $issue.body
    }
    
    # Add labels if present
    if (-not [string]::IsNullOrWhiteSpace($issue.labels)) {
        # Split labels by comma and add each one
        $labels = $issue.labels -split ',' | ForEach-Object { $_.Trim() }
        foreach ($label in $labels) {
            if (-not [string]::IsNullOrWhiteSpace($label)) {
                $args += "--label"
                $args += $label
            }
        }
    }
    
    # Add assignee if present
    if (-not [string]::IsNullOrWhiteSpace($issue.assignee)) {
        $args += "--assignee"
        $args += $issue.assignee
    }
    
    # Add milestone if present
    if (-not [string]::IsNullOrWhiteSpace($issue.milestone)) {
        $args += "--milestone"
        $args += $issue.milestone
    }
    
    # Execute command
    try {
        $output = & gh @args 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✓ Created successfully" -ForegroundColor Green
            $issuesCreated++
        } else {
            Write-Host "✗ Failed to create: $output" -ForegroundColor Red
            $issuesFailed++
        }
    }
    catch {
        Write-Host "✗ Failed to create: $_" -ForegroundColor Red
        $issuesFailed++
    }
    
    Write-Host ""
    
    # Small delay to avoid rate limiting
    Start-Sleep -Seconds 1
}

Write-Host ""
Write-Host "Import complete!" -ForegroundColor Green
Write-Host "Created: $issuesCreated issues"
if ($issuesFailed -gt 0) {
    Write-Host "Failed: $issuesFailed issues" -ForegroundColor Red
}
