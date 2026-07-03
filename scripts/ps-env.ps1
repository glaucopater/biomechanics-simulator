function Refresh-Path {
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [System.Environment]::GetEnvironmentVariable("Path", "User")
}

function Get-CMakeExe {
    Refresh-Path

    $cmd = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $candidates = @(
        "${env:ProgramFiles}\CMake\bin\cmake.exe",
        "${env:ProgramFiles(x86)}\CMake\bin\cmake.exe"
    )
    foreach ($path in $candidates) {
        if (Test-Path $path) { return $path }
    }

    throw @"
cmake not found. Install dependencies first:
  powershell -ExecutionPolicy Bypass -File .\setup-windows.ps1
Then open a new terminal, or re-run this script.
"@
}
