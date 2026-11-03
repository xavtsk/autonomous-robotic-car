param(
    [string]$Broker = '127.0.0.1',
    [ValidateRange(1, 65535)][int]$MqttPort = 1883,
    [ValidateRange(1, 65535)][int]$Port = 8765,
    [switch]$StartBroker,
    [switch]$Demo
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$python = Join-Path $repoRoot 'build\dashboard-venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    throw 'Create build/dashboard-venv with Python, then install tools/dashboard/requirements-windows.txt into it. See docs/DASHBOARD.md.'
}
$brokerProcess = $null
try {
    if ($StartBroker) {
        if ($Demo -or $Broker -ne '127.0.0.1' -or $MqttPort -ne 1883) {
            throw '-StartBroker requires live mode, -Broker 127.0.0.1 and -MqttPort 1883.'
        }
        $probe = [System.Net.Sockets.TcpClient]::new()
        try { $probe.Connect('127.0.0.1', 1883); $alreadyRunning = $true }
        catch { $alreadyRunning = $false }
        finally { $probe.Dispose() }
        if ($alreadyRunning) {
            Write-Host 'Port 1883 is already listening; using the existing broker.'
        } else {
            $mosquitto = Join-Path $repoRoot 'build\tooling\mosquitto\mosquitto.exe'
            if (-not (Test-Path -LiteralPath $mosquitto)) {
                $mosquitto = Join-Path $env:ProgramFiles 'mosquitto\mosquitto.exe'
            }
            if (-not (Test-Path -LiteralPath $mosquitto)) { throw 'Install Eclipse Mosquitto for Windows first. See docs/DASHBOARD.md.' }
            $logDir = Join-Path $repoRoot '.dashboard-data'
            New-Item -ItemType Directory -Force -Path $logDir | Out-Null
            $brokerProcess = Start-Process -FilePath $mosquitto -ArgumentList '-c config/mosquitto-dev.conf -v' `
                -WorkingDirectory $repoRoot -WindowStyle Hidden -PassThru `
                -RedirectStandardOutput (Join-Path $logDir 'broker-stdout.log') `
                -RedirectStandardError (Join-Path $logDir 'broker-stderr.log')
            Start-Sleep -Milliseconds 500
            if ($brokerProcess.HasExited) { throw "Broker failed; read $logDir\broker-stderr.log" }
            Write-Host 'Started local broker on TCP 1883. Pico MQTT_BROKER_IP must be this laptop''s hotspot/LAN IPv4 address.'
        }
    }
    $dashboardArgs = @((Join-Path $PSScriptRoot 'robot_dashboard.py'), '--port', "$Port", '--open')
    if ($Demo) { $dashboardArgs += '--demo' }
    else { $dashboardArgs += @('--broker', $Broker, '--mqtt-port', "$MqttPort", '--connect') }
    & $python @dashboardArgs
    if ($LASTEXITCODE -ne 0) { throw "Dashboard exited with code $LASTEXITCODE" }
} finally {
    if ($null -ne $brokerProcess -and -not $brokerProcess.HasExited) {
        Stop-Process -Id $brokerProcess.Id
    }
}
