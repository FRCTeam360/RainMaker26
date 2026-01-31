# GitHub Issues CSV Import Script (PowerShell)
# This script uses GitHub CLI (gh) to create issues from a CSV file
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login
# 3. Navigate to your repository directory or set $Repo variable below

param(
    [Parameter(Position=0)]
    [string]$CsvFile = "github_issues_to_add.csv",
    
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

# GitHub Project configuration for FRCTeam360 Rebuilt board
$ProjectOwner = "FRCTeam360"
$ProjectNumber = 2
$ProjectId = "PVT_kwDOAKOwYc4BMevW"
$EpicFieldId = "PVTSSF_lADOAKOwYc4BMevWzg73hPQ"
$PriorityFieldId = "PVTSSF_lADOAKOwYc4BMevWzg8Zqaw"

# Epic name to option ID mapping
$EpicOptions = @{
    "Architecture"        = "90ff87de"
    "Subsystems"          = "25133e18"
    "Automation"          = "52436123"
    "Autos"               = "5973533d"
    "Tooling"             = "dcae1a5f"
    "Testing/Prototyping" = "f16aa092"
    "Logging"             = "adddb420"
    "Simulations"         = "6c4f461c"
    "Vision"              = "778415d0"
    "Administration"      = "025c48c1"
    "Commands"            = "f1ebbe3d"
}

# Priority name to option ID mapping
$PriorityOptions = @{
    "Number One Highest Priority" = "18aec1f7"
    "High Priority"               = "a9977a8d"
    "Low Priority"                = "e9b3dc38"
}

Write-Host "Starting GitHub Issues import from: $CsvFile" -ForegroundColor Green
Write-Host ""

# Read CSV file
$issues = Import-Csv -Path $CsvFile

$issuesCreated = 0
$issuesFailed = 0
$successfulIssues = @()
$failedIssues = @()

# Determine the added issues file path (same directory as input CSV)
$csvDirectory = Split-Path -Parent $CsvFile
if ([string]::IsNullOrWhiteSpace($csvDirectory)) {
    $csvDirectory = "."
}
$addedCsvFile = Join-Path $csvDirectory "github_issues_added.csv"

foreach ($issue in $issues) {
    # Skip empty lines
    if ([string]::IsNullOrWhiteSpace($issue.title)) {
        continue
    }
    
    Write-Host "Creating issue: $($issue.title)" -ForegroundColor Yellow
    
    # Build gh issue create command arguments using ArrayList for better control
    $ghArgs = [System.Collections.ArrayList]@()
    [void]$ghArgs.Add("issue")
    [void]$ghArgs.Add("create")
    [void]$ghArgs.Add("--title")
    [void]$ghArgs.Add($issue.title)

    # Add repository if specified
    if (-not [string]::IsNullOrWhiteSpace($Repo)) {
        [void]$ghArgs.Add("--repo")
        [void]$ghArgs.Add($Repo)
    }

    # Add body (required for non-interactive mode, use space if empty)
    [void]$ghArgs.Add("--body")
    if (-not [string]::IsNullOrWhiteSpace($issue.body)) {
        [void]$ghArgs.Add($issue.body)
    } else {
        [void]$ghArgs.Add(" ")
    }

    # Add labels if present
    if (-not [string]::IsNullOrWhiteSpace($issue.labels)) {
        # Split labels by comma and add each one
        $labels = $issue.labels -split ',' | ForEach-Object { $_.Trim() }
        foreach ($label in $labels) {
            if (-not [string]::IsNullOrWhiteSpace($label)) {
                [void]$ghArgs.Add("--label")
                [void]$ghArgs.Add($label)
            }
        }
    }

    # Add assignee if present
    if (-not [string]::IsNullOrWhiteSpace($issue.assignee)) {
        [void]$ghArgs.Add("--assignee")
        [void]$ghArgs.Add($issue.assignee)
    }

    # Add milestone if present
    if (-not [string]::IsNullOrWhiteSpace($issue.milestone)) {
        [void]$ghArgs.Add("--milestone")
        [void]$ghArgs.Add($issue.milestone)
    }
    
    # Execute command
    try {
        $output = & gh @ghArgs 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "[OK] Created successfully" -ForegroundColor Green
            $issuesCreated++
            $successfulIssues += $issue

            # Add to project board - $output contains the issue URL
            $issueUrl = $output.Trim()
            $projectResult = gh project item-add $ProjectNumber --owner $ProjectOwner --url $issueUrl --format json 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  -> Added to Rebuilt board" -ForegroundColor Cyan

                # Parse item ID from JSON response (needed for custom fields)
                $itemData = $projectResult | ConvertFrom-Json
                $itemId = $itemData.id

                # Set Epic field if specified
                if (-not [string]::IsNullOrWhiteSpace($issue.epic)) {
                    $epicName = $issue.epic.Trim()
                    if ($EpicOptions.ContainsKey($epicName)) {
                        $epicOptionId = $EpicOptions[$epicName]

                        $epicResult = gh project item-edit --project-id $ProjectId --id $itemId --field-id $EpicFieldId --single-select-option-id $epicOptionId 2>&1
                        if ($LASTEXITCODE -eq 0) {
                            Write-Host "  -> Epic set to: $epicName" -ForegroundColor Cyan
                        } else {
                            Write-Host "  -> Warning: Could not set Epic: $epicResult" -ForegroundColor Yellow
                        }
                    } else {
                        Write-Host "  -> Warning: Unknown Epic '$epicName'. Valid options: $($EpicOptions.Keys -join ', ')" -ForegroundColor Yellow
                    }
                }

                # Set Priority field if specified
                if (-not [string]::IsNullOrWhiteSpace($issue.priority)) {
                    $priorityName = $issue.priority.Trim()
                    if ($PriorityOptions.ContainsKey($priorityName)) {
                        $priorityOptionId = $PriorityOptions[$priorityName]

                        $priorityResult = gh project item-edit --project-id $ProjectId --id $itemId --field-id $PriorityFieldId --single-select-option-id $priorityOptionId 2>&1
                        if ($LASTEXITCODE -eq 0) {
                            Write-Host "  -> Priority set to: $priorityName" -ForegroundColor Cyan
                        } else {
                            Write-Host "  -> Warning: Could not set Priority: $priorityResult" -ForegroundColor Yellow
                        }
                    } else {
                        Write-Host "  -> Warning: Unknown Priority '$priorityName'. Valid options: $($PriorityOptions.Keys -join ', ')" -ForegroundColor Yellow
                    }
                }
            } else {
                Write-Host "  -> Warning: Could not add to project: $projectResult" -ForegroundColor Yellow
            }
        } else {
            Write-Host "[FAIL] Failed to create: $output" -ForegroundColor Red
            $issuesFailed++
            $failedIssues += $issue
        }
    }
    catch {
        Write-Host "[FAIL] Failed to create: $_" -ForegroundColor Red
        $issuesFailed++
        $failedIssues += $issue
    }
    
    Write-Host ""
    
    # Small delay to avoid rate limiting
    Start-Sleep -Seconds 1
}

# Move successful issues to github_issues_added.csv
if ($successfulIssues.Count -gt 0) {
    # Check if added file exists to determine if we need headers
    $addedFileExists = Test-Path $addedCsvFile

    if ($addedFileExists) {
        # Append without headers
        $successfulIssues | Export-Csv -Path $addedCsvFile -Append -NoTypeInformation
    } else {
        # Create new file with headers
        $successfulIssues | Export-Csv -Path $addedCsvFile -NoTypeInformation
    }
    Write-Host "Moved $($successfulIssues.Count) issue(s) to: $addedCsvFile" -ForegroundColor Cyan
}

# Rewrite original CSV with only failed issues (keeps them for retry)
if ($failedIssues.Count -gt 0) {
    $failedIssues | Export-Csv -Path $CsvFile -NoTypeInformation
    Write-Host "Kept $($failedIssues.Count) failed issue(s) in: $CsvFile" -ForegroundColor Yellow
} elseif ($successfulIssues.Count -gt 0) {
    # All issues succeeded - write empty CSV with just headers
    "title,body,labels,assignee,milestone,epic,priority" | Set-Content -Path $CsvFile
    Write-Host "Cleared original CSV (all issues imported successfully)" -ForegroundColor Green
}

Write-Host ""
Write-Host "Import complete!" -ForegroundColor Green
Write-Host "Created: $issuesCreated issues"
if ($issuesFailed -gt 0) {
    Write-Host "Failed: $issuesFailed issues" -ForegroundColor Red
}
