param([string]$MsysRoot = 'C:\msys64')

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compiler = Join-Path $MsysRoot 'ucrt64\bin\gcc.exe'
$testDir = Join-Path $repoRoot 'build\host-tests'
$testExe = Join-Path $testDir 'test_ir_reflectance.exe'
New-Item -ItemType Directory -Force -Path $testDir | Out-Null
$originalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $MsysRoot 'ucrt64\bin');$originalPath"
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic `
        "-I$repoRoot\app_program\subsystem3" `
        "$repoRoot\app_program\subsystem3\ir_reflectance.c" `
        "$repoRoot\testing\subsystem3\test_ir_reflectance.c" -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'IR host test compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'IR host tests failed.' }
} finally {
    $env:PATH = $originalPath
}
