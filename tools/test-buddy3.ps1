param([string]$MsysRoot = 'C:\msys64', [switch]$CompileOnly)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compiler = Join-Path $MsysRoot 'ucrt64\bin\gcc.exe'
. (Join-Path $PSScriptRoot 'host-test-evidence.ps1')
$HostTestEvidence = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'buddy3' -CompileOnly ([bool]$CompileOnly)
$testDir = $HostTestEvidence.Directory
$hostTestFailure = $null
$originalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $MsysRoot 'ucrt64\bin');$originalPath"
    $childArguments = @('-ExecutionPolicy', 'Bypass', '-File',
                        "$repoRoot\tools\test-ir-reflectance.ps1", '-MsysRoot', $MsysRoot)
    if ($CompileOnly) { $childArguments += '-CompileOnly' }
    $HostTestExitCode = Invoke-HostTestNative -Evidence $HostTestEvidence `
        -FilePath 'powershell.exe' -ArgumentList $childArguments -Kind child
    if ($HostTestExitCode -ne 0) { throw 'Reflectance tests failed.' }
    $sources = @('ir_acquisition', 'ir_calibration', 'ir_reflectance', 'line_detection', 'line_controller', 'bc_decoder', 'line_navigation') |
        ForEach-Object { "$repoRoot\app_program\subsystem3\$_.c" }
    $testExe = Join-Path $testDir 'test_buddy3.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" @sources `
        "$repoRoot\testing\subsystem3\test_core_main.c" `
        "$repoRoot\testing\subsystem3\test_navigation.c" -o $testExe
    if ($HostTestExitCode -ne 0) { throw 'Buddy 3 host compilation failed.' }
    Invoke-RecordedHostExecutable $testExe
    if ($HostTestExitCode -ne 0) { throw 'Buddy 3 host tests failed.' }
    $recordExe = Join-Path $testDir 'test_barcode_record.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\scan_record_main.c" -o $recordExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode record test compilation failed.' }
    Invoke-RecordedHostExecutable $recordExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode record tests failed.' }
    $fastExe = Join-Path $testDir 'test_barcode_fast.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\fast_scan_main.c" -o $fastExe
    if ($HostTestExitCode -ne 0) { throw 'Fast barcode test compilation failed.' }
    Invoke-RecordedHostExecutable $fastExe
    if ($HostTestExitCode -ne 0) { throw 'Fast barcode tests failed.' }
    $faintExe = Join-Path $testDir 'test_barcode_faint.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\test_barcode_faint_main.c" -o $faintExe
    if ($HostTestExitCode -ne 0) { throw 'Faint barcode test compilation failed.' }
    Invoke-RecordedHostExecutable $faintExe
    if ($HostTestExitCode -ne 0) { throw 'Faint barcode tests failed.' }
    $varyingExe = Join-Path $testDir 'test_barcode_varying.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\variable_scan_main.c" -o $varyingExe
    if ($HostTestExitCode -ne 0) { throw 'Varying barcode correction test compilation failed.' }
    Invoke-RecordedHostExecutable $varyingExe
    if ($HostTestExitCode -ne 0) { throw 'Varying barcode correction tests failed.' }
    $scoreExe = Join-Path $testDir 'test_barcode_fit_error.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        -DBC_DECODER_FIT_TEST "-I$repoRoot\app_program\subsystem3" `
        "-I$repoRoot\testing\subsystem3" "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\fit_score_main.c" -o $scoreExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode fit-score test compilation failed.' }
    Invoke-RecordedHostExecutable $scoreExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode fit-score tests failed.' }
    $quietExe = Join-Path $testDir 'test_barcode_quiet.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\testing\subsystem3\quiet_zone_main.c" -o $quietExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode quiet-clock test compilation failed.' }
    Invoke-RecordedHostExecutable $quietExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode quiet-clock tests failed.' }
    $quietRecordExe = Join-Path $testDir 'test_barcode_quiet_record.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\record_zone_main.c" -o $quietRecordExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode quiet-record test compilation failed.' }
    Invoke-RecordedHostExecutable $quietRecordExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode quiet-record tests failed.' }
    $incrementalExe = Join-Path $testDir 'test_barcode_incremental.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        -DBC_DECODER_WORK_TEST -DS3_BARCODE_SYNC_REFERENCE `
        "-I$repoRoot\app_program\subsystem3" `
        "-I$repoRoot\testing\subsystem3" "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\work_quota_main.c" -o $incrementalExe
    if ($HostTestExitCode -ne 0) { throw 'Incremental barcode test compilation failed.' }
    Invoke-RecordedHostExecutable $incrementalExe
    if ($HostTestExitCode -ne 0) { throw 'Incremental barcode tests failed.' }
    $wideExe = Join-Path $testDir 'test_barcode_wide.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\wide_bar_main.c" -o $wideExe
    if ($HostTestExitCode -ne 0) { throw 'Wide barcode test compilation failed.' }
    Invoke-RecordedHostExecutable $wideExe
    if ($HostTestExitCode -ne 0) { throw 'Wide barcode tests failed.' }
    $replayExe = Join-Path $testDir 'replay_barcode_record.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -O2 `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\bc_decoder.c" `
        "$repoRoot\testing\subsystem3\replay_barcode_record_main.c" -o $replayExe
    if ($HostTestExitCode -ne 0) { throw 'Barcode replay compilation failed.' }
    $mappingExe = Join-Path $testDir 'test_ir_mapping.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" @sources `
        "$repoRoot\testing\subsystem3\test_mapping_main.c" -o $mappingExe
    if ($HostTestExitCode -ne 0) { throw 'IR mapping test compilation failed.' }
    Invoke-RecordedHostExecutable $mappingExe
    if ($HostTestExitCode -ne 0) { throw 'IR mapping tests failed.' }
    $driveExe = Join-Path $testDir 'test_s3_drive.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem3" "-I$repoRoot\app_program\subsystem2" "-I$repoRoot\app_program\controller" @sources `
        "$repoRoot\app_program\controller\s3_drive.c" `
        "$repoRoot\app_program\subsystem2\ctrl_motion.c" `
        "$repoRoot\app_program\subsystem2\motion_pack.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_drive_main.c" -o $driveExe
    if ($HostTestExitCode -ne 0) { throw 'Integration test compilation failed.' }
    Invoke-RecordedHostExecutable $driveExe
    if ($HostTestExitCode -ne 0) { throw 'Integration tests failed.' }
    $feedbackExe = Join-Path $testDir 'test_motion_feedback.exe'
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic -g `
        "-I$repoRoot\app_program\subsystem2" "-I$repoRoot\app_program\controller" `
        -include "$repoRoot\testing\subsystem3\motion_kernel_stub.h" `
        "$repoRoot\app_program\subsystem2\motion.c" `
        "$repoRoot\app_program\subsystem2\ctrl_motion.c" `
        "$repoRoot\app_program\subsystem2\motion_pack.c" `
        "$repoRoot\app_program\subsystem2\motion_estimation.c" `
        "$repoRoot\app_program\subsystem2\speed_pid.c" `
        "$repoRoot\testing\subsystem3\test_motion_feedback_main.c" -o $feedbackExe
    if ($HostTestExitCode -ne 0) { throw 'Motion feedback test compilation failed.' }
    Invoke-RecordedHostExecutable $feedbackExe
    if ($HostTestExitCode -ne 0) { throw 'Motion feedback tests failed.' }
} catch {
    $hostTestFailure = $_.Exception.Message
    throw
} finally {
    $env:PATH = $originalPath
    Complete-HostTestEvidence -Evidence $HostTestEvidence -Failure $hostTestFailure
}
