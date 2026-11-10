param(
    [string]$Compiler = 'clang.exe',
    [string]$WslDistribution = 'Ubuntu-22.04'
)
$ErrorActionPreference = 'Stop'
$clang = Get-Command -Name $Compiler -CommandType Application -ErrorAction SilentlyContinue
if (-not $clang) {
    throw "Clang not found: $Compiler. Put clang.exe on PATH or pass -Compiler with its full path."
}
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
$fixture = Join-Path $PSScriptRoot 'sensor_host'
$build = Join-Path $projectRoot 'tmp\sensor-host-tests'
New-Item -ItemType Directory -Path $build -Force | Out-Null
# Compile Linux objects on Windows, then link and run them under WSL.
& $clang.Source --target=x86_64-unknown-linux-gnu -std=c++14 -O1 -Wall -Wextra -Werror `
    -fno-exceptions -fno-rtti -fno-stack-protector -nostdinc `
    -isystem (Join-Path $fixture 'host_c') -I $fixture `
    -DARDUINO_UNOR4_WIFI -DNO_USB -c (Join-Path $fixture 'sensor_tests.cpp') `
    -o (Join-Path $build 'sensor_tests.o')
if ($LASTEXITCODE -ne 0) { throw 'Sensor host compilation failed.' }
& $clang.Source --target=x86_64-unknown-linux-gnu -c (Join-Path $fixture 'linux_entry.S') `
    -o (Join-Path $build 'entry.o')
if ($LASTEXITCODE -ne 0) { throw 'Host entry compilation failed.' }
function Convert-ToWslPath([string]$path) {
    $converted = & wsl -d $WslDistribution -- wslpath -a $path
    if ($LASTEXITCODE -ne 0) { throw "WSL path conversion failed: $path" }
    return $converted.Trim()
}
$linuxBuild = Convert-ToWslPath $build
& wsl -d $WslDistribution -- ld -o "$linuxBuild/sensor-tests" `
    "$linuxBuild/entry.o" "$linuxBuild/sensor_tests.o" `
    -dynamic-linker /lib64/ld-linux-x86-64.so.2 /lib/x86_64-linux-gnu/libc.so.6
if ($LASTEXITCODE -ne 0) { throw 'Sensor host linking failed.' }
$testOutput = & wsl -d $WslDistribution -- "$linuxBuild/sensor-tests"
$testExit = $LASTEXITCODE
$testOutput | Write-Output
if ($testExit -ne 0) { throw 'Sensor runtime checks failed.' }
# Pass firmware-generated CSV rows through the host's serial decoder.
$rowsPath = Join-Path $build 'csv-fixtures.txt'
$rows = @($testOutput | Where-Object { $_.StartsWith('CSV_FIXTURE:') } | ForEach-Object { $_.Substring(12) })
[IO.File]::WriteAllLines($rowsPath, $rows, [Text.Encoding]::ASCII)
$python = Join-Path $projectRoot '.venv\Scripts\python.exe'
$parserCheck = @'
import sys
sys.path.insert(0, sys.argv[1])
from sensor_telemetry import parse_line, LineDecoder
with open(sys.argv[2], encoding="ascii") as source:
    rows = source.read().splitlines()
assert len(rows) == 8
for row in rows:
    reading = parse_line(row, 1.0)
    assert reading is not None, row
    assert reading.ir == (0, 1, 0, 1, 1)
    assert (reading.distance_cm is None) == (reading.status != "OK")
stream = "# motor ACK OK 1\n" + "\n# motor UNREFERENCED 1 0 0 0 0 OK 1 0 -1\n".join(rows) + "\n"
decoder = LineDecoder()
received = []
for byte in stream.encode("ascii"):
    received.extend(decoder.feed(bytes([byte]), 1.0))
assert len(received) == 8
print("Python accepted 8 firmware CSV rows and skipped motor comments.")
'@
& $python -c $parserCheck (Join-Path $projectRoot 'code') $rowsPath
if ($LASTEXITCODE -ne 0) { throw 'Telemetry parser compatibility check failed.' }
Write-Output 'Firmware simulation and CSV compatibility checks passed.'
