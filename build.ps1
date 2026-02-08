# Kill running simulator so the linker can overwrite the exe, then build Release.
$exe = "biomechanics_simulator.exe"
Get-Process -Name ($exe -replace '\.exe$','') -ErrorAction SilentlyContinue | Stop-Process -Force
Set-Location $PSScriptRoot\build
cmake --build . --config Release
exit $LASTEXITCODE
