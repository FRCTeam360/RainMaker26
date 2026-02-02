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
    [string]$CsvFile = "scripts\github_issues\csv_exports\archive\issues_to_update_20260201_pt2 copy 2.csv",
    
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

# Function to get project configuration dynamically
function Get-ProjectConfig {
    param(
        [string]$ProjectNumber,
        [string]$ProjectOwner
    )
    
    Write-Host "Fetching project configuration from GitHub..." -ForegroundColor Cyan
    
    try {
        # Get project details
        $projectInfo = gh project view $ProjectNumber --owner $ProjectOwner --format json 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Error: Could not fetch project information" -ForegroundColor Red
            Write-Host "Details: $projectInfo" -ForegroundColor Red
            return $null
        }
        
        $project = $projectInfo | ConvertFrom-Json
        $projectId = $project.id
        
        Write-Host "  -> Project ID: $projectId" -ForegroundColor Cyan
        
        # Get all fields in the project
        $fieldsJson = gh project field-list $ProjectNumber --owner $ProjectOwner --format json 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Error: Could not fetch project fields" -ForegroundColor Red
            Write-Host "Details: $fieldsJson" -ForegroundColor Red
            return $null
        }
        
        Write-Host "  -> Raw fields response:" -ForegroundColor Gray
        Write-Host $fieldsJson -ForegroundColor Gray
        
        # Handle both single object and array responses
        $fieldsData = $fieldsJson | ConvertFrom-Json
        if ($null -eq $fieldsData) {
            Write-Host "Error: No fields returned from project" -ForegroundColor Red
            return $null
        }
        
        # The response might have a 'fields' property or be an array directly
        if ($fieldsData.PSObject.Properties.Name -contains "fields") {
            $fieldsList = @($fieldsData.fields)
        } elseif ($fieldsData -is [System.Collections.IEnumerable] -and $fieldsData -isnot [string]) {
            $fieldsList = @($fieldsData)
        } else {
            $fieldsList = @($fieldsData)
        }
        
        Write-Host "  -> Found $($fieldsList.Count) field(s)" -ForegroundColor Cyan
        
        $config = @{
            ProjectId = $projectId
            Fields = @{}
        }
        
        foreach ($field in $fieldsList) {
            if ($null -eq $field) { 
                Write-Host "    - Skipping null field" -ForegroundColor Gray
                continue 
            }
            
            # Skip fields without a name
            if ([string]::IsNullOrWhiteSpace($field.name)) {
                Write-Host "    - Skipping field with empty name" -ForegroundColor Gray
                continue
            }
            
            $fieldConfig = @{
                Id = $field.id
                Name = $field.name
                Type = $field.type
                Options = @{}
            }
            
            Write-Host "    - Field: $($field.name) (Type: $($field.type))" -ForegroundColor Gray
            
            # Collect options for single-select fields (both ProjectV2SingleSelectField and other types with options)
            if (($field.type -eq "single_select" -or $field.type -eq "ProjectV2SingleSelectField") -and $null -ne $field.options) {
                $optionsList = @($field.options)
                foreach ($option in $optionsList) {
                    if ($null -ne $option -and -not [string]::IsNullOrWhiteSpace($option.name)) {
                        $fieldConfig.Options[$option.name] = $option.id
                        Write-Host "      * $($option.name)" -ForegroundColor Gray
                    }
                }
            }
            
            $config.Fields[$field.name] = $fieldConfig
        }
        
        if ($config.Fields.Count -eq 0) {
            Write-Host "Warning: No valid fields found in project" -ForegroundColor Yellow
        }
        
        return $config
    }
    catch {
        Write-Host "Error fetching project config: $_" -ForegroundColor Red
        Write-Host "Stack trace: $($_.ScriptStackTrace)" -ForegroundColor Red
        return $null
    }
}

# Function to get CSV schema
function Get-CsvSchema {
    param([string]$CsvFile)
    
    try {
        $csv = Import-Csv -Path $CsvFile
        if ($csv.Count -gt 0) {
            return $csv[0].PSObject.Properties.Name
        }
    }
    catch {
        Write-Host "Error reading CSV schema: $_" -ForegroundColor Red
    }
    return $null
}

# Get dynamic project configuration
$ProjectConfig = Get-ProjectConfig -ProjectNumber $ProjectNumber -ProjectOwner $ProjectOwner
if ($null -eq $ProjectConfig) {
    Write-Host "Error: Could not retrieve project configuration. Exiting." -ForegroundColor Red
    exit 1
}

$ProjectId = $ProjectConfig.ProjectId

# Extract field IDs and options
$EpicField = $ProjectConfig.Fields["Epic"]
$PriorityField = $ProjectConfig.Fields["Priority"]

$EpicFieldId = $null
$PriorityFieldId = $null
$EpicOptions = @{}
$PriorityOptions = @{}

if ($null -eq $EpicField) {
    Write-Host "Warning: 'Epic' field not found in project" -ForegroundColor Yellow
} else {
    $EpicFieldId = $EpicField.Id
    $EpicOptions = $EpicField.Options
}

if ($null -eq $PriorityField) {
    Write-Host "Warning: 'Priority' field not found in project" -ForegroundColor Yellow
} else {
    $PriorityFieldId = $PriorityField.Id
    $PriorityOptions = $PriorityField.Options
}

# Get CSV schema
$CsvSchema = Get-CsvSchema -CsvFile $CsvFile
Write-Host "CSV Schema: $($CsvSchema -join ', ')" -ForegroundColor Cyan

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
    if (-not [string]::IsNullOrWhiteSpace($issue.epic) -and $null -ne $EpicFieldId) {
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
    if (-not [string]::IsNullOrWhiteSpace($issue.priority) -and $null -ne $PriorityFieldId) {
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
