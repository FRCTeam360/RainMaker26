# Branch Archiving Guide

## Overview

Instead of keeping old branches cluttering your repository, you can archive them using Git tags. This keeps your branch list clean while preserving the ability to restore branches in the future.

## Why Archive Branches?

- Keeps your branch list clean and manageable
- Preserves branch history for future reference
- Takes up minimal space (tags are just pointers)
- Easier to navigate when you have dozens of old branches

## Quick Start

The easiest way to archive a branch:

```powershell
.\scripts\github_branches\Archive-GitBranch.ps1 <branch-name>
```

**Example:**
```powershell
.\scripts\github_branches\Archive-GitBranch.ps1 Conversion-Factors
```

View all archived branches:
```bash
git tag -l "archive/*"
```

Restore an archived branch:
```bash
git checkout -b <branch-name> archive/<branch-name>
```

## How to Archive a Branch

### Step 1: Create an Archive Tag

```bash
git tag archive/<branch-name> <branch-name>
```

If the branch only exists on the remote, use:
```bash
git tag archive/<branch-name> remotes/origin/<branch-name>
```

**Example:**
```bash
git tag archive/VisionSubsystemWork remotes/origin/VisionSubsystemWork
```

### Step 2: Push the Tag to Remote

```bash
git push origin archive/<branch-name>
```

**Example:**
```bash
git push origin archive/VisionSubsystemWork
```

### Step 3: Delete the Remote Branch

```bash
git push -d origin <branch-name>
```

**Example:**
```bash
git push -d origin VisionSubsystemWork
```

### Step 4 (Optional): Delete the Local Branch

If the branch exists locally, delete it:

```bash
git branch -d <branch-name>
```

**Example:**
```bash
git branch -d VisionSubsystemWork
```

## Restoring an Archived Branch

If you need to work with an archived branch again, you can restore it:

```bash
git checkout -b <branch-name> archive/<branch-name>
```

**Example:**
```bash
git checkout -b VisionSubsystemWork archive/VisionSubsystemWork
```

This creates a new local branch from the archived tag.

## All-in-One Example

To archive the `VisionSubsystemWork` branch in one go:

```bash
# Create tag
git tag archive/VisionSubsystemWork remotes/origin/VisionSubsystemWork

# Push tag
git push origin archive/VisionSubsystemWork

# Delete remote branch
git push -d origin VisionSubsystemWork

# Delete local branch (if it exists)
git branch -d VisionSubsystemWork
```

## Automation

### Using a Git Alias

Add this to your Git config to create a custom command:

```bash
git config --global alias.archive-branch '!git tag archive/$1 remotes/origin/$1 && git push origin archive/$1 && git push -d origin $1'
```

Then use it like:
```bash
git archive-branch VisionSubsystemWork
```

### Using a PowerShell Script

Create a file called `Archive-GitBranch.ps1`:

```powershell
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
```

Then run it:
```powershell
.\Archive-GitBranch.ps1 VisionSubsystemWork
```

### Using a Bash Script (for WSL/Git Bash)

Create a file called `archive-branch.sh`:

```bash
#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./archive-branch.sh <branch-name>"
    exit 1
fi

BRANCH=$1

echo "Archiving branch: $BRANCH..."

git tag archive/$BRANCH remotes/origin/$BRANCH || { echo "Error: Could not create tag"; exit 1; }
git push origin archive/$BRANCH || { echo "Error: Could not push tag"; exit 1; }
git push -d origin $BRANCH || { echo "Error: Could not delete remote branch"; exit 1; }

echo "✓ Successfully archived $BRANCH"
```

Then run it:
```bash
chmod +x archive-branch.sh
./archive-branch.sh VisionSubsystemWork
```

## Notes

- Tags are cheap pointers—archiving doesn't consume significant space
- You can view all archive tags with: `git tag -l "archive/*"`
- Archive tags protect the branch history from accidental cleanup (git gc won't remove tagged commits)
- The tag name convention `archive/<branch-name>` makes it easy to identify archived branches
