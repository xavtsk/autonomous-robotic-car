param(
    [string]$Port = ''
)

$ErrorActionPreference = 'Stop'
$available = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
if ([string]::IsNullOrWhiteSpace($Port)) {
    if ($available.Count -eq 1) {
        $Port = $available[0]
    } else {
        $portList = if ($available.Count -eq 0) { '(none)' } else { $available -join ', ' }
        Write-Host "Available ports: $portList"
        Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name | Format-Table
        throw 'Connect the flashed Pico, then run .\tools\read-ir.ps1 -Port COMx with its actual port number.'
    }
}

$serial = New-Object System.IO.Ports.SerialPort $Port, 115200, None, 8, One
$serial.ReadTimeout = 1000
$serial.DtrEnable = $true
$serial.RtsEnable = $true
try {
    $serial.Open()
    Write-Host "Reading $Port. Press Ctrl+C to stop. Hold white paper or black tape under the connected sensor."
    while ($true) {
        try {
            Write-Host ($serial.ReadLine().TrimEnd())
        } catch [System.TimeoutException] {
            # Allow Ctrl+C even when the device is quiet.
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
