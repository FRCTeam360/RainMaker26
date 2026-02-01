param([string]$BranchName)

if (-not $BranchName) {
    Write-Host "Usage: .\Archive-GitBranch.ps1 <branch-name>"
    exit 1
}

Write-Host "Archiving branch: $BranchName..."

# Create tag
git tag archive/$BranchName remotes/origin/$BranchName
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Could not create tag for $BranchName"
    exit 1
}

# Push tag
git push origin archive/$BranchName
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Could not push tag"
    exit 1
}

# Delete remote branch
git push -d origin $BranchName
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Could not delete remote branch"
    exit 1
}

Write-Host "Successfully archived $BranchName"
