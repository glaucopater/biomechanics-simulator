# From repo root: configure if needed, kill simulator, build Release, then run.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$buildDir = Join-Path $root "build"

. (Join-Path $root "scripts\ps-env.ps1")
$cmake = Get-CMakeExe

Get-Process -Name "biomechanics_simulator" -ErrorAction SilentlyContinue | Stop-Process -Force

if (-not (Test-Path (Join-Path $buildDir "CMakeCache.txt"))) {
    Write-Host "No build directory found. Running CMake configure ..."
    & $cmake -S $root -B $buildDir -G "Visual Studio 17 2022" -A x64
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Push-Location $buildDir
& $cmake --build . --config Release
if ($LASTEXITCODE -ne 0) { Pop-Location; exit $LASTEXITCODE }
Pop-Location

& $root\build\Release\biomechanics_simulator.exe @args
