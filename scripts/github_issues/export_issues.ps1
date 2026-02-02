# GitHub Issues CSV Export Script (PowerShell)
# This script uses GitHub CLI (gh) to export issues from a repository to a CSV file, optionally filtered by milestone
#
# Prerequisites:
# 1. Install GitHub CLI: choco install gh
# 2. Authenticate with: gh auth login
# 3. Navigate to your repository directory or set $Repo variable below

param(
    [Parameter(Position=0)]
    [string]$CsvFile = "",

    [Parameter(Position=1)]
    [string]$Repo = "FRCTeam360/RainMaker26",

    [string]$Milestone = "",

    [int]$ProjectNumber = 2,

    [string]$ProjectOwner = "FRCTeam360"
)

# Set default CSV file name with timestamp if not provided
if ([string]::IsNullOrWhiteSpace($CsvFile)) {
    $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $CsvFile = "scripts\github_issues\csv_exports\github_issues_export_$timestamp.csv"
}

# Ensure the csv_exports directory exists
$exportDir = Split-Path -Parent $CsvFile
if (-not (Test-Path $exportDir)) {
    New-Item -ItemType Directory -Path $exportDir -Force | Out-Null
    Write-Host "Created export directory: $exportDir" -ForegroundColor Cyan
}

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
            FieldIdToName = @{}
        }
        
        foreach ($field in $fieldsList) {
            if ($null -eq $field) { 
                continue 
            }
            
            # Skip fields without a name
            if ([string]::IsNullOrWhiteSpace($field.name)) {
                continue
            }
            
            $fieldConfig = @{
                Id = $field.id
                Name = $field.name
                Type = $field.type
                Options = @{}
                IdToOptionName = @{}
            }
            
            Write-Host "    - Field: $($field.name) (Type: $($field.type))" -ForegroundColor Gray
            
            # Collect options for single-select fields
            if (($field.type -eq "single_select" -or $field.type -eq "ProjectV2SingleSelectField") -and $null -ne $field.options) {
                $optionsList = @($field.options)
                foreach ($option in $optionsList) {
                    if ($null -ne $option -and -not [string]::IsNullOrWhiteSpace($option.name)) {
                        $fieldConfig.Options[$option.name] = $option.id
                        $fieldConfig.IdToOptionName[$option.id] = $option.name
                        Write-Host "      * $($option.name)" -ForegroundColor Gray
                    }
                }
            }
            
            $config.Fields[$field.name] = $fieldConfig
            $config.FieldIdToName[$field.id] = $field.name
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

# Get dynamic project configuration
$ProjectConfig = Get-ProjectConfig -ProjectNumber $ProjectNumber -ProjectOwner $ProjectOwner
if ($null -eq $ProjectConfig) {
    Write-Host "Error: Could not retrieve project configuration. Exiting." -ForegroundColor Red
    exit 1
}

$ProjectId = $ProjectConfig.ProjectId

Write-Host "Starting GitHub Issues export to: $CsvFile" -ForegroundColor Green
Write-Host ""

# Build gh issue list command
$ghArgs = @("issue", "list", "--limit", "1000", "--state", "all", "--json", "number,title,body,labels,assignees,milestone,url,state,author,createdAt,updatedAt,closedAt,comments")

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
        exit 1P
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
$fieldValueMaps = @{}

# Initialize maps for all single-select fields
foreach ($fieldName in $ProjectConfig.Fields.Keys) {
    $fieldValueMaps[$fieldName] = @{}
}

foreach ($item in $projectItems.items) {
    if ($item.content.type -eq "Issue") {
        # Get issue number directly from content
        $issueNumber = $item.content.number.ToString()
        $issueNumbers += $issueNumber

        # Get values for all single-select fields dynamically
        foreach ($fieldName in $ProjectConfig.Fields.Keys) {
            $field = $ProjectConfig.Fields[$fieldName]
            
            # Only process single-select fields with options
            if (($field.Type -eq "single_select" -or $field.Type -eq "ProjectV2SingleSelectField") -and $field.Options.Count -gt 0) {
                # Try to get the field value from the item
                # Field names in items might be lowercase or have different casing
                $fieldValue = $null
                
                # Check if the item has this field (case-insensitive)
                foreach ($prop in $item.PSObject.Properties) {
                    if ($prop.Name -eq $fieldName) {
                        $fieldValue = $prop.Value
                        break
                    }
                }
                
                if ($fieldValue) {
                    $fieldValueMaps[$fieldName][$issueNumber] = $fieldValue
                }
            }
        }
    } else {
      # print out non-issue items
      Write-Host "Warning: Non-issue item found in board: $($item.content.type)" -ForegroundColor Yellow
    }
}

Write-Host "Found $($issueNumbers.Count) issues in the Rebuilt board" -ForegroundColor Cyan

# Find and output missing issues (in repo but not in Rebuilt board, and still open)
$missingIssues = $issues | Where-Object { $_.number.ToString() -notin $issueNumbers -and $_.state -eq "OPEN" }
Write-Host "Missing issues (not in Rebuilt board and still open):" -ForegroundColor Yellow
$missingIssues | ForEach-Object { Write-Host "Issue #$($_.number): $($_.title)" -ForegroundColor Yellow }

# Fetch full issue details for each issue number
$issues = @()
foreach ($issueNumber in $issueNumbers) {
    Write-Host "Fetching details for issue #$issueNumber..." -ForegroundColor Yellow

    $ghArgs = @("issue", "view", $issueNumber, "--json", "number,title,body,labels,assignees,milestone,url,state,author,createdAt,updatedAt,closedAt,comments")

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

    # Format assignees as comma-separated string (all assignees)
    $assigneesStr = ""
    if ($issue.assignees -and $issue.assignees.Count -gt 0) {
        $assigneesStr = ($issue.assignees | ForEach-Object { $_.login }) -join ","
    }

    # Get milestone title
    $milestoneStr = ""
    if ($issue.milestone) {
        $milestoneStr = $issue.milestone.title
    }

    # Get author
    $authorStr = ""
    if ($issue.author) {
        $authorStr = $issue.author.login
    }

    # Format dates
    $createdAtStr = ""
    if ($issue.createdAt) {
        $createdAtStr = $issue.createdAt
    }

    $updatedAtStr = ""
    if ($issue.updatedAt) {
        $updatedAtStr = $issue.updatedAt
    }

    $closedAtStr = ""
    if ($issue.closedAt) {
        $closedAtStr = $issue.closedAt
    }

    # Get comment count
    $commentCount = 0
    if ($issue.comments) {
        $commentCount = $issue.comments
    }

    # Create base CSV object with all standard GitHub fields
    $csvObject = [PSCustomObject]@{
        number     = $issue.number
        title      = $issue.title
        body       = $issue.body -replace "`r`n", " " -replace "`n", " "  # Replace newlines with spaces
        state      = $issue.state
        url        = $issue.url
        author     = $authorStr
        labels     = $labelsStr
        assignees  = $assigneesStr
        milestone  = $milestoneStr
        createdAt  = $createdAtStr
        updatedAt  = $updatedAtStr
        closedAt   = $closedAtStr
        comments   = $commentCount
    }

    # Dynamically add all single-select project fields
    foreach ($fieldName in $ProjectConfig.Fields.Keys) {
        $field = $ProjectConfig.Fields[$fieldName]
        
        # Only add single-select fields with options
        if (($field.Type -eq "single_select" -or $field.Type -eq "ProjectV2SingleSelectField") -and $field.Options.Count -gt 0) {
            $fieldValue = ""
            if ($fieldValueMaps[$fieldName].ContainsKey($issue.number.ToString())) {
                $fieldValue = $fieldValueMaps[$fieldName][$issue.number.ToString()]
            }
            
            # Add the field as a property to the CSV object
            $csvObject | Add-Member -MemberType NoteProperty -Name $fieldName.ToLower() -Value $fieldValue -Force
        }
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
