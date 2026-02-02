param(
    [int]$DaysInactive = 3
)

Write-Host "Finding stale branches (inactive for $DaysInactive+ days, 0 commits ahead of main, no active PRs)..." -ForegroundColor Cyan
Write-Host ""

# Get current date
$cutoffDate = (Get-Date).AddDays(-$DaysInactive)

# Get all remote branches except main
$branches = git branch -r --format='%(refname:short)' | Where-Object { $_ -notmatch 'origin/(main|HEAD)' }

$staleBranches = @()

foreach ($branch in $branches) {
    $branchName = $branch -replace 'origin/', ''
    
    # Get last commit date
    $lastCommitDate = git log -1 --format='%ci' $branch 2>$null
    if (-not $lastCommitDate) {
        continue
    }
    
    $commitDate = [DateTime]::Parse($lastCommitDate)
    
    # Check if inactive
    if ($commitDate -gt $cutoffDate) {
        continue
    }
    
    # Check if 0 commits ahead of main
    $aheadCount = git rev-list --count origin/main..$branch 2>$null
    if ($aheadCount -ne "0") {
        continue
    }
    
    # Check for PRs
    $prs = gh pr list --head $branchName --state all --json number,state 2>$null | ConvertFrom-Json
    
    # Determine PR status
    $mergedPR = $prs | Where-Object { $_.state -eq "MERGED" } | Select-Object -First 1
    $openPR = $prs | Where-Object { $_.state -eq "OPEN" } | Select-Object -First 1
    $closedPR = $prs | Where-Object { $_.state -eq "CLOSED" } | Select-Object -First 1
    
    if ($mergedPR) {
        $prInfo = "#$($mergedPR.number) Merged"
    } elseif ($openPR) {
        $prInfo = "#$($openPR.number) Open"
    } elseif ($closedPR) {
        $prInfo = "#$($closedPR.number) Closed"
    } else {
        $prInfo = "No PR"
    }
    
    # This branch is stale
    $daysOld = [math]::Round(((Get-Date) - $commitDate).TotalDays, 0)
    $staleBranches += [PSCustomObject]@{
        Branch = $branchName
        LastCommit = $commitDate.ToString("yyyy-MM-dd")
        DaysOld = $daysOld
        PR = $prInfo
    }
}

if ($staleBranches.Count -eq 0) {
    Write-Host "No stale branches found!" -ForegroundColor Green
    exit 0
}

# Display results
Write-Host "Found $($staleBranches.Count) stale branch(es):" -ForegroundColor Yellow
Write-Host ""
$staleBranches | Sort-Object -Property DaysOld -Descending | Format-Table -AutoSize

Write-Host ""
Write-Host "To archive these branches, run:" -ForegroundColor Cyan
foreach ($branch in $staleBranches) {
    Write-Host "  .\Archive-GitBranch.ps1 $($branch.Branch)" -ForegroundColor White
}
