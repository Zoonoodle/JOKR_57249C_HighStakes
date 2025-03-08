# Auto Git Push Script
# Automatically commits and pushes changes to GitHub

param (
    [string]$commitMessage = "Auto-update: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
)

# Change to the project directory (Modify this path if necessary)
$projectPath = "$PSScriptRoot"  # Automatically sets to the script's folder
Set-Location -Path $projectPath

# Ensure Git is initialized
if (!(Test-Path .git)) {
    Write-Host "Git is not initialized. Initializing..."
    git init
    git remote add origin https://github.com/Zoonoodle/JOKR_57249C_HighStakes.git
}

# Add all changes
git add .

# Commit with a timestamp message
git commit -m "$commitMessage"

# Push to GitHub
git push origin main

Write-Host "✅ Changes pushed successfully!"
