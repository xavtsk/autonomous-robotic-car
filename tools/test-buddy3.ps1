param([string]$MsysRoot = 'C:\msys64')
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compiler = Join-Path $MsysRoot 'ucrt64\bin\gcc.exe'
$testDir = Join-Path $repoRoot 'build\host-tests'
New-Item -ItemType Directory -Force -Path $testDir | Out-Null
$originalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $MsysRoot 'ucrt64\bin');$originalPath"
    & powershell -ExecutionPolicy Bypass -File "$repoRoot\tools\test-ir-reflectance.ps1" -MsysRoot $MsysRoot
    if ($LASTEXITCODE -ne 0) { throw 'Reflectance tests failed.' }
    $sources = @('ir_acquisition', 'ir_calibration', 'ir_reflectance', 'line_detection', 'line_controller', 'barcode_decoder', 'line_navigation') |
        ForEach-Object { "$repoRoot\app_program\subsystem3\$_.c" }
    $testExe = Join-Path $testDir 'test_buddy3.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" @sources `
        "$repoRoot\testing\subsystem3\test_core.c" -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Buddy 3 host compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Buddy 3 host tests failed.' }
} finally { $env:PATH = $originalPath }
