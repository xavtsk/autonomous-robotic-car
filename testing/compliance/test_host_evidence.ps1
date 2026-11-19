param([string]$Python = "$PSScriptRoot\..\..\build\dashboard-venv\Scripts\python.exe")

# Exercises logging with an existing interpreter. No generated C binary runs.
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
. (Join-Path $repoRoot 'tools\host-test-evidence.ps1')
$Python = (Get-Item -LiteralPath $Python).FullName

function Assert-Equal($Actual, $Expected) {
    if ($Actual -cne $Expected) { throw "Expected [$Expected], observed [$Actual]." }
}

$evidence = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'evidence-tool-check' -CompileOnly $false
$other = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'evidence-tool-check' -CompileOnly $true
if ($evidence.Directory -eq $other.Directory) { throw 'Output directories collided.' }
$program = Join-Path $evidence.Directory 'fake native.py'
@'
import json
from pathlib import Path
import sys
mode = sys.argv[1]
if mode == 'build':
    Path(sys.argv[sys.argv.index('-o') + 1]).write_bytes(b'test artifact, not executable')
elif mode == 'arguments':
    print(json.dumps(sys.argv[2:]))
    print('stderr retained', file=sys.stderr)
elif mode == 'fail':
    print('intentional failure', file=sys.stderr)
    sys.exit(7)
elif mode == 'mutate':
    Path(sys.argv[2]).write_text('mutated disposable fixture')
'@ | Set-Content -LiteralPath $program -Encoding UTF8
$artifact = Join-Path $evidence.Directory 'not executable.bin'
Assert-Equal (Invoke-HostTestNative $evidence $Python @($program, 'build', '-o', $artifact) build) 0
$literalArguments = @('path with spaces', "apostrophe's", 'literal$dollar', 'literal`tick')
Assert-Equal (Invoke-HostTestNative $evidence $Python (@($program, 'arguments') + $literalArguments) test) 0
$argumentLog = Get-Content -LiteralPath (Join-Path $evidence.Directory '002-test.log') -Raw
if ($argumentLog -notmatch 'stderr retained') { throw 'stderr was lost.' }
$jsonLine = @($argumentLog -split '\r?\n' | Where-Object { $_.StartsWith('[') })[0]
Assert-Equal (($jsonLine | ConvertFrom-Json) -join '|') ($literalArguments -join '|')
Complete-HostTestEvidence $evidence
Assert-Equal ((Get-Content -LiteralPath (Join-Path $evidence.Directory 'results.json') -Raw | ConvertFrom-Json).status) 'PASS'
$firstReceipt = (Get-FileHash -LiteralPath (Join-Path $evidence.Directory 'results.json')).Hash

Assert-Equal (Invoke-HostTestNative $other $Python @($program, 'build', '-o', (Join-Path $other.Directory 'unused.bin')) build) 0
Assert-Equal (Invoke-HostTestNative $other 'nonexistent-test.exe' @() test) 0
Complete-HostTestEvidence $other
Assert-Equal ((Get-Content -LiteralPath (Join-Path $other.Directory 'results.json') -Raw | ConvertFrom-Json).status) 'COMPILE_PASS_EXECUTION_NOT_RUN'
Assert-Equal $other.Steps[1].status 'SKIP_COMPILE_ONLY'
Assert-Equal ((Get-FileHash -LiteralPath (Join-Path $evidence.Directory 'results.json')).Hash) $firstReceipt

$failure = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'evidence-tool-check' -CompileOnly $false
Assert-Equal (Invoke-HostTestNative $failure $Python @($program, 'fail') test) 7
Complete-HostTestEvidence $failure
Assert-Equal ((Get-Content -LiteralPath (Join-Path $failure.Directory 'results.json') -Raw | ConvertFrom-Json).status) 'FAIL'

$startup = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'evidence-tool-check' -CompileOnly $false
Complete-HostTestEvidence $startup 'intentional startup failure'
Assert-Equal ((Get-Content -LiteralPath (Join-Path $startup.Directory 'results.json') -Raw | ConvertFrom-Json).status) 'FAIL'

$changed = New-HostTestEvidence -RepoRoot $repoRoot -Suite 'evidence-tool-check' -CompileOnly $false
$temporarySource = Join-Path $changed.Directory 'disposable.c'
'original disposable fixture' | Set-Content -LiteralPath $temporarySource -Encoding UTF8
$caught = $false
try {
    Invoke-HostTestNative $changed $Python @($program, 'mutate', $temporarySource) test | Out-Null
} catch {
    $caught = $_.Exception.Message -like 'Source input changed during measurement:*'
}
if (-not $caught) { throw 'Source mutation did not fail the receipt.' }
Complete-HostTestEvidence $changed
Assert-Equal ((Get-Content -LiteralPath (Join-Path $changed.Directory 'results.json') -Raw | ConvertFrom-Json).status) 'FAIL'

Write-Host 'Evidence helper checks passed; no generated C program executed.'
