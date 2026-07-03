# Install build requirements on Windows (Git, CMake, MSVC via VS 2022 Build Tools).
# GLFW, ImGui, and Jolt are fetched automatically by CMake on first configure.
#
# Run from repo root (elevates to Administrator when needed):
#   powershell -ExecutionPolicy Bypass -File .\setup-windows.ps1

#Requires -Version 5.1

$ErrorActionPreference = "Stop"

function Test-Command([string] $Name) {
    return $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

function Refresh-Path {
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [System.Environment]::GetEnvironmentVariable("Path", "User")
}

function Test-MsvcInstalled {
    if (Test-Command cl) { return $true }

    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $false }

    $installPath = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath 2>$null
    return [bool]$installPath
}

function Install-WingetPackage {
    param(
        [Parameter(Mandatory = $true)][string] $Id,
        [string[]] $OverrideArgs
    )

    $installed = winget list --id $Id --accept-source-agreements 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Already installed: $Id"
        return
    }

    Write-Host "Installing: $Id ..."
    $wingetArgs = @(
        "install", "--id", $Id, "-e",
        "--accept-package-agreements", "--accept-source-agreements"
    )
    if ($OverrideArgs) {
        $wingetArgs += "--override"
        $wingetArgs += ($OverrideArgs -join " ")
    }

    & winget @wingetArgs
    if ($LASTEXITCODE -ne 0) {
        throw "winget install failed for $Id (exit $LASTEXITCODE)"
    }
}

function Ensure-Administrator {
    $isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator
    )
    if ($isAdmin) { return }

    Write-Host "Administrator rights are required for Visual Studio Build Tools."
    Write-Host "Re-launching elevated ..."
    $argList = "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    Start-Process powershell.exe -Verb RunAs -ArgumentList $argList | Out-Null
    exit 0
}

Ensure-Administrator

if (-not (Test-Command winget)) {
    Write-Error @"
winget is not available. Install 'App Installer' from the Microsoft Store, then re-run:
  powershell -ExecutionPolicy Bypass -File .\setup-windows.ps1
"@
    exit 1
}

Write-Host "=== Biomechanics Simulator - Windows setup ===" -ForegroundColor Cyan
Write-Host ""

Install-WingetPackage -Id "Git.Git"
Install-WingetPackage -Id "Kitware.CMake"

if (-not (Test-MsvcInstalled)) {
    Write-Host "Installing Visual Studio 2022 Build Tools (C++ workload). This can take several minutes ..."
    Install-WingetPackage -Id "Microsoft.VisualStudio.2022.BuildTools" -OverrideArgs @(
        "--wait",
        "--passive",
        "--add", "Microsoft.VisualStudio.Workload.VCTools",
        "--includeRecommended"
    )
} else {
    Write-Host "Already installed: MSVC (Visual Studio C++ tools)"
}

Refresh-Path

Write-Host ""
Write-Host "=== Verification ===" -ForegroundColor Cyan

$checks = @(
    @{ Name = "git";   Ok = (Test-Command git) },
    @{ Name = "cmake"; Ok = (Test-Command cmake) },
    @{ Name = "cl";    Ok = (Test-MsvcInstalled) }
)

$allOk = $true
foreach ($check in $checks) {
    $status = if ($check.Ok) { "OK" } else { "MISSING" }
    $color = if ($check.Ok) { "Green" } else { "Red" }
    Write-Host ("  {0,-8} {1}" -f $check.Name, $status) -ForegroundColor $color
    if (-not $check.Ok) { $allOk = $false }
}

Write-Host ""
if (-not $allOk) {
    Write-Warning "Some tools are still missing. Close this terminal, open a new one, and run setup again."
    exit 1
}

Write-Host "Setup complete." -ForegroundColor Green
Write-Host ""
Write-Host 'Next steps (new terminal recommended):'
Write-Host '  cmake -B build -G "Visual Studio 17 2022" -A x64'
Write-Host '  cmake --build build --config Release'
Write-Host '  .\build\Release\biomechanics_simulator.exe'
Write-Host ''
Write-Host 'Or use: powershell -ExecutionPolicy Bypass -File .\build-and-run.ps1'
Write-Host '(after the first cmake -B build configure step)'
