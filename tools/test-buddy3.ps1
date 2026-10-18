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
        "$repoRoot\testing\subsystem3\test_core.c" `
        "$repoRoot\testing\subsystem3\test_navigation.c" -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Buddy 3 host compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Buddy 3 host tests failed.' }
    $driveExe = Join-Path $testDir 'test_s3_drive.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" "-I$repoRoot\app_program\subsystem2" @sources `
        "$repoRoot\app_program\subsystem3\s3_drive.c" `
        "$repoRoot\app_program\subsystem2\motion_controller.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_drive.c" -o $driveExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed.' }
    & $driveExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration tests failed.' }
    $feedbackExe = Join-Path $testDir 'test_motion_feedback.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem2" `
        -include "$repoRoot\testing\subsystem3\motion_kernel_stub.h" `
        "$repoRoot\app_program\subsystem2\motion.c" `
        "$repoRoot\app_program\subsystem2\motion_controller.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_motion_feedback.c" -o $feedbackExe
    if ($LASTEXITCODE -ne 0) { throw 'Motion feedback test compilation failed.' }
    & $feedbackExe
    if ($LASTEXITCODE -ne 0) { throw 'Motion feedback tests failed.' }
} finally { $env:PATH = $originalPath }
