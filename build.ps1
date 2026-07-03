# Kill running simulator, configure if needed, then build Release.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$buildDir = Join-Path $root "build"

. (Join-Path $root "scripts\ps-env.ps1")
$cmake = Get-CMakeExe

$exe = "biomechanics_simulator.exe"
Get-Process -Name ($exe -replace '\.exe$','') -ErrorAction SilentlyContinue | Stop-Process -Force

if (-not (Test-Path (Join-Path $buildDir "CMakeCache.txt"))) {
    Write-Host "No build directory found. Running CMake configure ..."
    & $cmake -S $root -B $buildDir -G "Visual Studio 17 2022" -A x64
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Set-Location $buildDir
& $cmake --build . --config Release
exit $LASTEXITCODE
