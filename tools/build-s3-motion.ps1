param(
    [string]$MsysRoot = 'C:\msys64'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$compilerBin = Join-Path $repoRoot 'build\tooling\arm\bin'
$sdkRoot = Join-Path $repoRoot 'build\tooling\pico-sdk'
$bashPath = Join-Path $MsysRoot 'usr\bin\bash.exe'
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
    & $bashPath -c 'make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk S3_MOTION=1 IR_BENCH=0 WIFI=none TEST=0 -j8'
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed (exit $LASTEXITCODE). Do not flash an older file."
    }
    & "$PSScriptRoot\check-buddy3-profile.ps1" -Profile motion
    Write-Host "Firmware: $repoRoot\build_make\mtk3pico_smp1_usb_cdc_s3_motion.uf2"
} finally {
    $env:PATH = $originalPath
    Pop-Location
}
