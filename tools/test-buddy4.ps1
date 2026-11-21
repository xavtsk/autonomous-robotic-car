param(
    [string]$MsysRoot = 'C:\msys64',
    [switch]$CompileOnly
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compilerDir = Join-Path $MsysRoot 'ucrt64\bin'
$compiler = Join-Path $compilerDir 'gcc.exe'
. (Join-Path $PSScriptRoot 'host-test-evidence.ps1')
$HostTestEvidence = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'buddy4' -CompileOnly ([bool]$CompileOnly)
$testDir = $HostTestEvidence.Directory
$testExe = Join-Path $testDir 'test_buddy4.exe'
$hostTestFailure = $null
$originalPath = $env:PATH
try {
    if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
        throw "GCC was not found at $compiler. Supply -MsysRoot with your MSYS2 installation folder."
    }
    $env:PATH = "$compilerDir;$originalPath"
    $sources = @('imu_sensor', 'imu_calibration', 'tilt',
                 'hump_detection', 'events_motion', 'terr_data') |
        ForEach-Object { Join-Path $repoRoot "app_program\subsystem4\$_.c" }
    # Never define NDEBUG: the test's assertions must execute.
    Invoke-RecordedHostCompiler -std=c11 -O2 -Wall -Wextra -Werror -pedantic `
        "-I$repoRoot\app_program\subsystem4" `
        "$repoRoot\testing\subsystem4\imu_host_main.c" @sources -lm -o $testExe
    if ($HostTestExitCode -ne 0) { throw 'Buddy 4 test compilation failed.' }
    Invoke-RecordedHostExecutable $testExe
    if ($HostTestExitCode -ne 0) { throw "Buddy 4 tests failed (exit $HostTestExitCode)." }
} catch {
    $hostTestFailure = $_.Exception.Message
    throw
} finally {
    $env:PATH = $originalPath
    Complete-HostTestEvidence -Evidence $HostTestEvidence -Failure $hostTestFailure
}
