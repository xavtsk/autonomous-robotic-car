param(
    [string]$MsysRoot = 'C:\msys64',
    [switch]$CompileOnly
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compilerDir = Join-Path $MsysRoot 'ucrt64\bin'
$compiler = Join-Path $compilerDir 'gcc.exe'
$testDir = Join-Path $repoRoot 'build\host-tests\buddy4'
$testExe = Join-Path $testDir 'test_buddy4.exe'
$logPath = Join-Path $testDir 'latest.log'
if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
    throw "GCC was not found at $compiler. Supply -MsysRoot with your MSYS2 installation folder."
}
New-Item -ItemType Directory -Force -Path $testDir | Out-Null
$originalPath = $env:PATH
try {
    $env:PATH = "$compilerDir;$originalPath"
    "Buddy 4 host tests: $(Get-Date -Format o)" | Set-Content -LiteralPath $logPath
    $sources = @('imu_sensor', 'imu_calibration', 'tilt_estimation',
                 'hump_detection', 'motion_events', 'terrain_monitor') |
        ForEach-Object { Join-Path $repoRoot "app_program\subsystem4\$_.c" }
    # Never define NDEBUG: the test's assertions must execute.
    & $compiler -std=c11 -O2 -Wall -Wextra -Werror -pedantic `
        "-I$repoRoot\app_program\subsystem4" `
        "$repoRoot\testing\subsystem4\host_test.c" @sources -lm -o $testExe 2>&1 |
        Tee-Object -FilePath $logPath -Append
    if ($LASTEXITCODE -ne 0) { throw 'Buddy 4 test compilation failed.' }
    'Compilation PASS' | Tee-Object -FilePath $logPath -Append
    if ($CompileOnly) {
        'Execution NOT RUN (-CompileOnly). No runtime pass claimed.' |
            Tee-Object -FilePath $logPath -Append
    } else {
        & $testExe 2>&1 | Tee-Object -FilePath $logPath -Append
        if ($LASTEXITCODE -ne 0) {
            throw "Buddy 4 tests failed (exit $LASTEXITCODE). Read the assertion/error in $logPath."
        }
        'Execution PASS: all assertions completed.' | Tee-Object -FilePath $logPath -Append
    }
} catch {
    "ERROR: $($_.Exception.Message)" | Add-Content -LiteralPath $logPath
    throw
} finally {
    $env:PATH = $originalPath
    Write-Host "Log: $logPath"
}
