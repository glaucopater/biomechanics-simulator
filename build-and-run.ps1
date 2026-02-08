# From repo root: kill simulator, build Release, then run.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

Get-Process -Name "biomechanics_simulator" -ErrorAction SilentlyContinue | Stop-Process -Force

Push-Location $root\build
cmake --build . --config Release
if ($LASTEXITCODE -ne 0) { Pop-Location; exit $LASTEXITCODE }
Pop-Location

& $root\build\Release\biomechanics_simulator.exe @args
