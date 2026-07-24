<#
Reset script for OwnTech environment-setup video dry runs.

Uninstalls Git, CMake, VS Code (via winget) and removes leftover config/cache
folders (PlatformIO core, VS Code settings, gitconfig, pip cache) so the next
recording pass starts from a genuinely clean machine.

Python is intentionally NOT auto-uninstalled — versions vary and you don't
want this script guessing which Python install to remove. It's listed for
you to remove manually with one printed command.

Usage (elevated PowerShell):
    Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
    .\reset_environment.ps1

Only run this on the disposable test machine used for recording, not your
everyday dev machine — it uninstalls real tooling.
#>

#Requires -RunAsAdministrator

$ErrorActionPreference = 'Continue'

$appsToRemove = @(
    'Git.Git',
    'Kitware.CMake',
    'Microsoft.VisualStudioCode'
)

$foldersToDelete = @(
    "$env:USERPROFILE\.platformio",
    "$env:APPDATA\Code",
    "$env:USERPROFILE\.vscode",
    "$env:LOCALAPPDATA\pip"
)

Write-Host "== OwnTech environment reset ==" -ForegroundColor Cyan
Write-Host "This will uninstall: $($appsToRemove -join ', ')"
Write-Host "And delete these folders if present:"
$foldersToDelete | ForEach-Object { Write-Host "  $_" }
Write-Host "  $env:USERPROFILE\.gitconfig"
Write-Host ""
$projectPath = Read-Host "Path to your cloned project folder to also delete (leave blank to skip)"
Write-Host ""
$confirm = Read-Host "Type YES to continue"
if ($confirm -ne 'YES') {
    Write-Host "Aborted." -ForegroundColor Yellow
    exit 0
}

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Write-Warning "winget not found. Install 'App Installer' from the Microsoft Store, or uninstall these apps manually from Settings > Apps."
} else {
    foreach ($id in $appsToRemove) {
        Write-Host "Uninstalling $id..." -ForegroundColor Cyan
        winget uninstall --id $id -e --silent --accept-source-agreements
    }

    Write-Host ""
    Write-Host "Checking for Python 3.x installs (not auto-removed)..." -ForegroundColor Cyan
    $pythonLines = winget list --name Python 2>$null | Select-String 'Python 3'
    if ($pythonLines) {
        Write-Host "Found the following — remove manually if you want a clean Python re-install too:"
        $pythonLines | ForEach-Object { Write-Host "  $_" }
        Write-Host "Example: winget uninstall --id Python.Python.3.12 -e"
    } else {
        Write-Host "No winget-managed Python 3.x install found."
    }
}

Write-Host ""
foreach ($folder in $foldersToDelete) {
    if (Test-Path $folder) {
        Write-Host "Deleting $folder..." -ForegroundColor Cyan
        Remove-Item -Path $folder -Recurse -Force -ErrorAction SilentlyContinue
    }
}

$gitconfig = "$env:USERPROFILE\.gitconfig"
if (Test-Path $gitconfig) {
    Write-Host "Deleting $gitconfig..." -ForegroundColor Cyan
    Remove-Item -Path $gitconfig -Force -ErrorAction SilentlyContinue
}

if ($projectPath -and (Test-Path $projectPath)) {
    Write-Host "Deleting project folder $projectPath..." -ForegroundColor Cyan
    Remove-Item -Path $projectPath -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "== PATH entries worth a manual glance ==" -ForegroundColor Cyan
$env:Path -split ';' | Where-Object { $_ -match 'git|python|cmake|vs ?code' -and $_ -ne '' }

Write-Host ""
Write-Host "Done. Reboot before your next recording pass so PATH changes and file locks fully clear." -ForegroundColor Green
