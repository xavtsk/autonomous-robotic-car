param([ValidateSet('bench', 'motion', 'normal')][string]$Profile)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$nm = Join-Path $root 'build/tooling/arm/bin/arm-none-eabi-nm.exe'
function Undefined([string]$Path) {
    $result = & $nm -u (Join-Path $root "build_make/mtkernel_3/app_program/$Path.o")
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect $Path" }
    return @($result | ForEach-Object { ($_ -split '\s+')[-1] })
}
$main = Undefined 'app_main'
$s3 = Undefined 'subsystem3/subsystem3'
$runtime = Undefined 'controller/s3_integration'
if ($Profile -eq 'bench') {
    if ($main -notcontains 'subsystem3_start' -or
        ($main -match '^subsystem[1245]_(init|start)$') -or
        ($s3 -match '^(motion_|motor_|stop$|s3_integration_)') -or $runtime.Count) {
        throw 'Stationary bench startup boundary failed.'
    }
} elseif ($Profile -eq 'motion') {
    if ($main -notcontains 'subsystem2_start' -or $main -notcontains 'subsystem3_start' -or
        ($main -match '^subsystem[145]_(init|start)$') -or
        $runtime -notcontains 'motion_get_report' -or $runtime -contains 'motion_poll') {
        throw 'Integrated controller ownership boundary failed.'
    }
} else {
    if ($s3.Count -or $runtime.Count) { throw 'Default profile unexpectedly starts Buddy 3.' }
}
foreach ($name in @('ir_acquisition', 'ir_calibration', 'ir_reflectance',
                    'line_detection', 'line_controller', 'barcode_decoder', 'line_navigation')) {
    if ((Undefined "subsystem3/$name") -match '^(motion_|motor_|stop$|tk_)') {
        throw "Portable sensor core depends on motion or RTOS: $name"
    }
}
Write-Host "PASS: $Profile startup and portable sensor boundary (object-symbol check)."
