# Single command: compile E2E and run from repo root.
# Usage: .\run-e2e.ps1
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

Push-Location $root\build
cmake --build . --config Release --target e2e_verification
if ($LASTEXITCODE -ne 0) { Pop-Location; exit $LASTEXITCODE }
Pop-Location

Push-Location $root
& .\build\tests\Release\e2e_verification.exe
$exitCode = $LASTEXITCODE
Pop-Location
exit $exitCode
