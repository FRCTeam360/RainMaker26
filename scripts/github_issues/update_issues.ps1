# GitHub Issues CSV Update Script (PowerShell)
# This script uses GitHub CLI (gh) to update existing issues from a CSV file
# Supports partial updates - only provide fields you want to change
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login
# 3. Navigate to your repository directory or set $Repo variable below
#
# CSV Format:
#   number,title,body,labels,assignee,milestone,epic,priority
#   123,New Title,,bug,username,,Architecture,High Priority
#
# Leave fields blank (empty) to skip updating them

param(
    [Parameter(Position=0)]
    [string]$CsvFile = "scripts\github_issues\github_issues_to_update.csv",
    
    [Parameter(Position=1)]
    [string]$Repo = "FRCTeam360/RainMaker26",

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

# Check if CSV file exists
if (-not (Test-Path $CsvFile)) {
    Write-Host "Error: CSV file '$CsvFile' not found." -ForegroundColor Red
    Write-Host "Usage: .\update_issues.ps1 [csv_file] [owner/repo]" -ForegroundColor Yellow
    exit 1
}

# GitHub Project configuration for FRCTeam360 Rebuilt board
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

Write-Host "Starting GitHub Issues update from: $CsvFile" -ForegroundColor Green
Write-Host ""

# Read CSV file
$issues = Import-Csv -Path $CsvFile

$issuesUpdated = 0
$issuesFailed = 0

foreach ($issue in $issues) {
    # Skip empty lines or missing issue numbers
    if ([string]::IsNullOrWhiteSpace($issue.number)) {
        Write-Host "Skipping row with missing issue number" -ForegroundColor Yellow
        continue
    }
    
    $issueNumber = $issue.number
    Write-Host "Updating issue #$issueNumber" -ForegroundColor Yellow
    
    # Build gh issue edit command arguments using ArrayList
    $ghArgs = [System.Collections.ArrayList]@()
    [void]$ghArgs.Add("issue")
    [void]$ghArgs.Add("edit")
    [void]$ghArgs.Add($issueNumber)

    # Add repository if specified
    if (-not [string]::IsNullOrWhiteSpace($Repo)) {
        [void]$ghArgs.Add("--repo")
        [void]$ghArgs.Add($Repo)
    }

    # Track if any fields are being updated
    $hasUpdates = $false

    # Add title if present
    if (-not [string]::IsNullOrWhiteSpace($issue.title)) {
        [void]$ghArgs.Add("--title")
        [void]$ghArgs.Add($issue.title)
        $hasUpdates = $true
    }

    # Add body if present
    if (-not [string]::IsNullOrWhiteSpace($issue.body)) {
        [void]$ghArgs.Add("--body")
        [void]$ghArgs.Add($issue.body)
        $hasUpdates = $true
    }

    # Add labels if present (replaces all labels)
    if (-not [string]::IsNullOrWhiteSpace($issue.labels)) {
        # Split labels by comma and add each one
        $labels = $issue.labels -split ',' | ForEach-Object { $_.Trim() }
        $labelsList = $labels -join ","
        [void]$ghArgs.Add("--label")
        [void]$ghArgs.Add($labelsList)
        $hasUpdates = $true
    }

    # Add assignee if present
    if (-not [string]::IsNullOrWhiteSpace($issue.assignee)) {
        [void]$ghArgs.Add("--assignee")
        [void]$ghArgs.Add($issue.assignee)
        $hasUpdates = $true
    }

    # Add milestone if present
    if (-not [string]::IsNullOrWhiteSpace($issue.milestone)) {
        [void]$ghArgs.Add("--milestone")
        [void]$ghArgs.Add($issue.milestone)
        $hasUpdates = $true
    }

    # Check if there are any updates to apply
    if (-not $hasUpdates) {
        Write-Host "  -> No standard fields to update, checking project fields only" -ForegroundColor Cyan
    } else {
        # Execute command
        try {
            $output = & gh @ghArgs 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  -> Issue fields updated successfully" -ForegroundColor Green
            } else {
                Write-Host "  -> Failed to update issue: $output" -ForegroundColor Red
                $issuesFailed++
                Write-Host ""
                continue
            }
        }
        catch {
            Write-Host "  -> Failed to update issue: $_" -ForegroundColor Red
            $issuesFailed++
            Write-Host ""
            continue
        }
    }

    # Update project board fields (Epic and Priority) if specified
    $projectFieldsUpdated = $false

    # Get issue URL for project operations
    $issueUrl = ""
    if (-not [string]::IsNullOrWhiteSpace($Repo)) {
        $issueUrl = "https://github.com/$Repo/issues/$issueNumber"
    } else {
        # Try to get URL from gh issue view
        $issueViewResult = gh issue view $issueNumber --json url 2>&1
        if ($LASTEXITCODE -eq 0) {
            $issueViewData = $issueViewResult | ConvertFrom-Json
            $issueUrl = $issueViewData.url
        }
    }

    # Get project item ID if we need to update Epic or Priority
    $itemId = $null
    if ((-not [string]::IsNullOrWhiteSpace($issue.epic)) -or (-not [string]::IsNullOrWhiteSpace($issue.priority))) {
        if (-not [string]::IsNullOrWhiteSpace($issueUrl)) {
            # Fetch project items to find the item ID for this issue
            try {
                $projectItemsJson = gh project item-list $ProjectNumber --owner $ProjectOwner --format json --limit 500 2>&1
                if ($LASTEXITCODE -eq 0) {
                    $projectItems = $projectItemsJson | ConvertFrom-Json
                    foreach ($item in $projectItems.items) {
                        if ($item.content.type -eq "Issue" -and $item.content.number -eq [int]$issueNumber) {
                            $itemId = $item.id
                            break
                        }
                    }

                    if ($null -eq $itemId) {
                        Write-Host "  -> Warning: Issue not found in Rebuilt board, cannot update Epic/Priority" -ForegroundColor Yellow
                    }
                }
            } catch {
                Write-Host "  -> Warning: Could not fetch project items: $_" -ForegroundColor Yellow
            }
        }
    }

    # Set Epic field if specified and item found
    if (-not [string]::IsNullOrWhiteSpace($issue.epic)) {
        if ($null -ne $itemId) {
            $epicName = $issue.epic.Trim()
            if ($EpicOptions.ContainsKey($epicName)) {
                $epicOptionId = $EpicOptions[$epicName]

                $epicResult = gh project item-edit --project-id $ProjectId --id $itemId --field-id $EpicFieldId --single-select-option-id $epicOptionId 2>&1
                if ($LASTEXITCODE -eq 0) {
                    Write-Host "  -> Epic set to: $epicName" -ForegroundColor Cyan
                    $projectFieldsUpdated = $true
                } else {
                    Write-Host "  -> Warning: Could not set Epic: $epicResult" -ForegroundColor Yellow
                }
            } else {
                Write-Host "  -> Warning: Unknown Epic '$epicName'. Valid options: $($EpicOptions.Keys -join ', ')" -ForegroundColor Yellow
            }
        }
    }

    # Set Priority field if specified and item found
    if (-not [string]::IsNullOrWhiteSpace($issue.priority)) {
        if ($null -ne $itemId) {
            $priorityName = $issue.priority.Trim()
            if ($PriorityOptions.ContainsKey($priorityName)) {
                $priorityOptionId = $PriorityOptions[$priorityName]

                $priorityResult = gh project item-edit --project-id $ProjectId --id $itemId --field-id $PriorityFieldId --single-select-option-id $priorityOptionId 2>&1
                if ($LASTEXITCODE -eq 0) {
                    Write-Host "  -> Priority set to: $priorityName" -ForegroundColor Cyan
                    $projectFieldsUpdated = $true
                } else {
                    Write-Host "  -> Warning: Could not set Priority: $priorityResult" -ForegroundColor Yellow
                }
            } else {
                Write-Host "  -> Warning: Unknown Priority '$priorityName'. Valid options: $($PriorityOptions.Keys -join ', ')" -ForegroundColor Yellow
            }
        }
    }

    # Count as successful update if any standard fields or project fields were updated
    if ($hasUpdates -or $projectFieldsUpdated) {
        $issuesUpdated++
    } else {
        Write-Host "  -> No changes made" -ForegroundColor Yellow
    }
    
    Write-Host ""
    
    # Small delay to avoid rate limiting
    Start-Sleep -Milliseconds 500
}

Write-Host ""
Write-Host "Update complete!" -ForegroundColor Green
Write-Host "Updated: $issuesUpdated issues"
if ($issuesFailed -gt 0) {
    Write-Host "Failed: $issuesFailed issues" -ForegroundColor Red
}
