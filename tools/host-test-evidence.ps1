# Dot-source this helper from a host-test script. It never flashes or runs a car.
function New-HostTestEvidence {
    param([string]$RepoRoot,
          [ValidatePattern('^[a-z0-9-]+$')][string]$Suite, [bool]$CompileOnly)
    $stamp = [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fffffff')
    $suffix = [Guid]::NewGuid().ToString('N').Substring(0, 8)
    $directory = Join-Path $RepoRoot "build\host-tests\$Suite\$stamp-$suffix"
    New-Item -ItemType Directory -Path $directory -ErrorAction Stop | Out-Null
    $revision = (& git -C $RepoRoot rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify host-test source revision.' }
    return [pscustomobject]@{
        Directory = $directory
        Revision = $revision
        PowerShellVersion = $PSVersionTable.PSVersion.ToString()
        CompileOnly = $CompileOnly
        StartedUtc = [DateTime]::UtcNow.ToString('o')
        Steps = [System.Collections.ArrayList]::new()
    }
}

function Invoke-HostTestNative {
    param($Evidence, [string]$FilePath, [string[]]$ArgumentList,
          [ValidateSet('build', 'test', 'child')][string]$Kind)
    $number = $Evidence.Steps.Count + 1
    $name = '{0:D3}-{1}' -f $number, $Kind
    $logPath = Join-Path $Evidence.Directory "$name.log"
    $step = [ordered]@{
        kind = $Kind
        executable = $FilePath
        arguments = @($ArgumentList)
        started_utc = [DateTime]::UtcNow.ToString('o')
        status = 'NOT_RUN'
        exit_code = $null
        log = "$name.log"
        input_sha256 = [ordered]@{}
    }
    foreach ($argument in $ArgumentList) {
        if ($argument -match '\.(c|h|ps1)$' -and (Test-Path -LiteralPath $argument -PathType Leaf)) {
            $step.input_sha256[$argument] = (Get-FileHash -LiteralPath $argument -Algorithm SHA256).Hash.ToLowerInvariant()
        }
    }
    $step | ConvertTo-Json -Depth 5 |
        Set-Content -LiteralPath (Join-Path $Evidence.Directory "$name.command.json") -Encoding UTF8
    try {
        if ($Kind -eq 'test' -and $Evidence.CompileOnly) {
            $step.status = 'SKIP_COMPILE_ONLY'
            'Execution NOT RUN (-CompileOnly).' |
                Set-Content -LiteralPath $logPath -Encoding UTF8
            Write-Host "Execution NOT RUN: $FilePath (-CompileOnly)."
            return 0
        }
        # Call-operator argument splatting preserves paths and individual options;
        # no shell command string or Start-Process argument re-quoting is used.
        $application = Get-Command -Name $FilePath -CommandType Application -ErrorAction Stop
        $step.resolved_executable = $application.Source
        $step.executable_sha256 = (Get-FileHash -LiteralPath $application.Source -Algorithm SHA256).Hash.ToLowerInvariant()
        $savedErrorPreference = $ErrorActionPreference
        try {
            # Windows PowerShell 5 presents redirected native stderr as an error
            # record. Preserve it and the actual exit code instead of aborting
            # solely because the process wrote a diagnostic to stderr.
            $ErrorActionPreference = 'Continue'
            $PSNativeCommandUseErrorActionPreference = $false
            $global:LASTEXITCODE = $null
            & $application.Source @ArgumentList 2>&1 |
                Tee-Object -FilePath $logPath |
                ForEach-Object { Write-Host $_ }
            $code = $global:LASTEXITCODE
        } finally {
            $ErrorActionPreference = $savedErrorPreference
        }
        if ($null -eq $code) { throw 'Native process did not return an exit code.' }
        $step.exit_code = $code
        $step.status = if ($code -eq 0) { 'PASS' } else { 'FAIL' }
        if ($Kind -eq 'build' -and $code -eq 0) {
            $outputIndex = [Array]::IndexOf($ArgumentList, '-o')
            if ($outputIndex -ge 0 -and ($outputIndex + 1) -lt $ArgumentList.Count) {
                $artifact = $ArgumentList[$outputIndex + 1]
                if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
                    throw "Compiler reported success without output: $artifact"
                }
                $step.artifact = $artifact
                $step.artifact_sha256 = (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash.ToLowerInvariant()
            }
        }
        return $code
    } catch {
        $step.status = 'ERROR'
        $step.error = $_.Exception.Message
        $step.error | Add-Content -LiteralPath $logPath -Encoding UTF8
        throw
    } finally {
        $sourceChanged = $false
        foreach ($inputPath in @($step.input_sha256.Keys)) {
            if (-not (Test-Path -LiteralPath $inputPath -PathType Leaf) -or
                (Get-FileHash -LiteralPath $inputPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $step.input_sha256[$inputPath]) {
                $step.status = 'ERROR'
                $step.error = "Source input changed during measurement: $inputPath"
                $sourceChanged = $true
            }
        }
        $step.finished_utc = [DateTime]::UtcNow.ToString('o')
        [void]$Evidence.Steps.Add([pscustomobject]$step)
        $step | ConvertTo-Json -Depth 5 |
            Set-Content -LiteralPath (Join-Path $Evidence.Directory "$name.result.json") -Encoding UTF8
        if ($sourceChanged) { throw $step.error }
    }
}

function Invoke-RecordedHostCompiler {
    $script:HostTestExitCode = Invoke-HostTestNative -Evidence $script:HostTestEvidence `
        -FilePath $script:compiler -ArgumentList @($args) -Kind build
}

function Invoke-RecordedHostExecutable {
    $program = [string]$args[0]
    $programArguments = @()
    if ($args.Count -gt 1) { $programArguments = @($args[1..($args.Count - 1)]) }
    $script:HostTestExitCode = Invoke-HostTestNative -Evidence $script:HostTestEvidence `
        -FilePath $program -ArgumentList $programArguments -Kind test
}

function Complete-HostTestEvidence {
    param($Evidence, [string]$Failure)
    $status = if ($Failure) { 'FAIL' } elseif ($Evidence.CompileOnly) {
        'COMPILE_PASS_EXECUTION_NOT_RUN'
    } else { 'PASS' }
    if ($Evidence.Steps.Count -eq 0 -and -not $Failure) { $status = 'INCOMPLETE' }
    $badSteps = @($Evidence.Steps | Where-Object {
        $_.status -ne 'PASS' -and
        -not ($Evidence.CompileOnly -and $_.kind -eq 'test' -and $_.status -eq 'SKIP_COMPILE_ONLY')
    })
    if ($badSteps.Count -gt 0) { $status = 'FAIL' }
    $builds = @($Evidence.Steps | Where-Object { $_.kind -eq 'build' -and $_.status -eq 'PASS' })
    if (-not $Failure -and $badSteps.Count -eq 0 -and $builds.Count -eq 0) { $status = 'INCOMPLETE' }
    [ordered]@{
        revision = $Evidence.Revision
        powershell_version = $Evidence.PowerShellVersion
        started_utc = $Evidence.StartedUtc
        finished_utc = [DateTime]::UtcNow.ToString('o')
        compile_only = $Evidence.CompileOnly
        status = $status
        failure = $Failure
        steps = @($Evidence.Steps)
        limits = @('Host results do not establish firmware or physical timing equivalence.',
                   'Input hashes cover explicit source arguments, not a complete transitive header inventory.')
    } | ConvertTo-Json -Depth 8 |
        Set-Content -LiteralPath (Join-Path $Evidence.Directory 'results.json') -Encoding UTF8
    Write-Host "Evidence: $($Evidence.Directory)"
}
