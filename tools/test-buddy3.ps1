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
    $recordExe = Join-Path $testDir 'test_barcode_record.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_record.c" -o $recordExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode record test compilation failed.' }
    & $recordExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode record tests failed.' }
    $faintExe = Join-Path $testDir 'test_barcode_faint.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_faint.c" -o $faintExe
    if ($LASTEXITCODE -ne 0) { throw 'Faint barcode test compilation failed.' }
    & $faintExe
    if ($LASTEXITCODE -ne 0) { throw 'Faint barcode tests failed.' }
    $varyingExe = Join-Path $testDir 'test_barcode_varying.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_varying.c" -o $varyingExe
    if ($LASTEXITCODE -ne 0) { throw 'Varying barcode correction test compilation failed.' }
    & $varyingExe
    if ($LASTEXITCODE -ne 0) { throw 'Varying barcode correction tests failed.' }
    $scoreExe = Join-Path $testDir 'test_barcode_fit_error.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\testing\subsystem3\test_barcode_fit_error.c" -o $scoreExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode fit-score test compilation failed.' }
    & $scoreExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode fit-score tests failed.' }
    $quietExe = Join-Path $testDir 'test_barcode_quiet.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\testing\subsystem3\test_barcode_quiet.c" -o $quietExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode quiet-clock test compilation failed.' }
    & $quietExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode quiet-clock tests failed.' }
    $quietRecordExe = Join-Path $testDir 'test_barcode_quiet_record.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_quiet_record.c" -o $quietRecordExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode quiet-record test compilation failed.' }
    & $quietRecordExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode quiet-record tests failed.' }
    $incrementalExe = Join-Path $testDir 'test_barcode_incremental.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\testing\subsystem3\test_barcode_incremental.c" -o $incrementalExe
    if ($LASTEXITCODE -ne 0) { throw 'Incremental barcode test compilation failed.' }
    & $incrementalExe
    if ($LASTEXITCODE -ne 0) { throw 'Incremental barcode tests failed.' }
    $wideExe = Join-Path $testDir 'test_barcode_wide.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_wide.c" -o $wideExe
    if ($LASTEXITCODE -ne 0) { throw 'Wide barcode test compilation failed.' }
    & $wideExe
    if ($LASTEXITCODE -ne 0) { throw 'Wide barcode tests failed.' }
    $replayExe = Join-Path $testDir 'replay_barcode_record.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\barcode_decoder.c" `
        "$repoRoot\testing\subsystem3\replay_barcode_record.c" -o $replayExe
    if ($LASTEXITCODE -ne 0) { throw 'Barcode replay compilation failed.' }
    $mappingExe = Join-Path $testDir 'test_ir_mapping.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" @sources `
        "$repoRoot\testing\subsystem3\test_mapping.c" -o $mappingExe
    if ($LASTEXITCODE -ne 0) { throw 'IR mapping test compilation failed.' }
    & $mappingExe
    if ($LASTEXITCODE -ne 0) { throw 'IR mapping tests failed.' }
    $driveExe = Join-Path $testDir 'test_s3_drive.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" "-I$repoRoot\app_program\subsystem2" "-I$repoRoot\app_program\controller" @sources `
        "$repoRoot\app_program\controller\s3_drive.c" `
        "$repoRoot\app_program\subsystem2\motion_controller.c" `
        "$repoRoot\app_program\subsystem2\motion_pack.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_drive.c" -o $driveExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed.' }
    & $driveExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration tests failed.' }
    $feedbackExe = Join-Path $testDir 'test_motion_feedback.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem2" "-I$repoRoot\app_program\controller" `
        -include "$repoRoot\testing\subsystem3\motion_kernel_stub.h" `
        "$repoRoot\app_program\subsystem2\motion.c" `
        "$repoRoot\app_program\subsystem2\motion_controller.c" `
        "$repoRoot\app_program\subsystem2\motion_pack.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_motion_feedback.c" -o $feedbackExe
    if ($LASTEXITCODE -ne 0) { throw 'Motion feedback test compilation failed.' }
    & $feedbackExe
    if ($LASTEXITCODE -ne 0) { throw 'Motion feedback tests failed.' }
} finally { $env:PATH = $originalPath }
