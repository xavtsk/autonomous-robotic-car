param(
    [string]$MsysRoot = 'C:\msys64',
    [switch]$Telemetry
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compilerBin = Join-Path $repoRoot 'build\tooling\arm\bin'
$sdkRoot = Join-Path $repoRoot 'build\tooling\pico-sdk'
$bashPath = Join-Path $MsysRoot 'usr\bin\bash.exe'
$networkFlags = 'WIFI=none NET=none MQTT=0'
$image = 'mtk3pico_smp1_usb_cdc_s3_motion.uf2'
if ($Telemetry) {
    if (-not (Test-Path -LiteralPath (Join-Path $repoRoot 'config\wifi_secrets.h'))) {
        throw 'Create config/wifi_secrets.h from wifi_secrets.example.h, then set the hotspot credentials and laptop MQTT_BROKER_IP. Do not commit it.'
    }
    foreach ($dependency in @('lib\cyw43-driver\src\cyw43_ctrl.c', 'lib\lwip\src\core\init.c')) {
        if (-not (Test-Path -LiteralPath (Join-Path $sdkRoot $dependency))) {
            throw 'Initialize SDK networking: git -C build/tooling/pico-sdk submodule update --init lib/cyw43-driver lib/lwip'
        }
    }
    $networkFlags = 'WIFI=cyw43 NET=lwip MQTT=1 IR_DEBUG=1'
    $image = 'mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_ir_debug.uf2'
}
foreach ($required in @($bashPath, (Join-Path $compilerBin 'arm-none-eabi-gcc.exe'),
                        (Join-Path $sdkRoot 'lib\tinyusb\src\tusb.c'))) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Missing build dependency: $required. See testing/subsystem3/README.md."
    }
}

$originalPath = $env:PATH
Push-Location $repoRoot
try {
    $env:PATH = "$compilerBin;$MsysRoot\ucrt64\bin;$MsysRoot\usr\bin;$originalPath"
    & $bashPath -c "make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk CONSOLE=usb_cdc S3_MOTION=1 IR_BENCH=0 $networkFlags TEST=0 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8"
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed (exit $LASTEXITCODE). Do not flash an older file."
    }
    & "$PSScriptRoot\check-buddy3-profile.ps1" -Profile motion
    Write-Host "Firmware: $repoRoot\build_make\$image"
} finally {
    $env:PATH = $originalPath
    Pop-Location
}
