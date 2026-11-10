param(
    [string]$ArduinoCli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
$includeDirectory = Join-Path $projectRoot 'code\firmware\grip_control'
$sketchDirectory = Join-Path $projectRoot 'tmp\grip_logic_tests'
$buildDirectory = Join-Path $projectRoot 'tmp\grip-logic-tests-build'
New-Item -ItemType Directory -Path $sketchDirectory -Force | Out-Null
# Copy production headers beside the sketch for the Arduino compiler.
foreach ($name in @('motor_logic.h', 'grip_config.h')) {
    $source = Join-Path $includeDirectory $name
    $copy = Join-Path $sketchDirectory $name
    Copy-Item -LiteralPath $source -Destination $copy -Force
    if ((Get-FileHash -LiteralPath $source).Hash -ne (Get-FileHash -LiteralPath $copy).Hash) {
        throw "Test staging hash mismatch: $name"
    }
}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'grip_logic_tests\grip_logic_tests.ino') `
    -Destination (Join-Path $sketchDirectory 'grip_logic_tests.ino') -Force
& $ArduinoCli compile --fqbn arduino:renesas_uno:unor4wifi --warnings all `
    --build-path $buildDirectory $sketchDirectory
if ($LASTEXITCODE -ne 0) { throw 'Compiler-evaluated grip logic tests failed.' }
Write-Output 'Grip logic assertions passed. No upload or hardware test performed.'
