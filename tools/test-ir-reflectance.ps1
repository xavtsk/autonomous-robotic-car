param([string]$MsysRoot = 'C:\msys64', [switch]$CompileOnly)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compiler = Join-Path $MsysRoot 'ucrt64\bin\gcc.exe'
. (Join-Path $PSScriptRoot 'host-test-evidence.ps1')
$HostTestEvidence = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'ir-reflectance' -CompileOnly ([bool]$CompileOnly)
$testDir = $HostTestEvidence.Directory
$hostTestFailure = $null
$testExe = Join-Path $testDir 'test_ir_reflectance.exe'
$originalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $MsysRoot 'ucrt64\bin');$originalPath"
    Invoke-RecordedHostCompiler -std=c11 -Wall -Wextra -Werror -pedantic `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\ir_reflectance.c" `
        "$repoRoot\testing\subsystem3\test_ir_reflectance_main.c" -o $testExe
    if ($HostTestExitCode -ne 0) { throw 'IR host test compilation failed.' }
    Invoke-RecordedHostExecutable $testExe
    if ($HostTestExitCode -ne 0) { throw 'IR host tests failed.' }
} catch {
    $hostTestFailure = $_.Exception.Message
    throw
} finally {
    $env:PATH = $originalPath
    Complete-HostTestEvidence -Evidence $HostTestEvidence -Failure $hostTestFailure
}
