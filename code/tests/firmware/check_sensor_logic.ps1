param(
    [string]$Compiler = 'C:\ProgramData\miniforge3\envs\ryzen-ai-1.7.0\Lib\site-packages\win64.o\tools\peano\bin\clang.exe',
    [string]$WslDistribution = 'Ubuntu-22.04'
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..'))
$fixture = Join-Path $PSScriptRoot 'sensor_host'
$build = Join-Path $projectRoot 'tmp\sensor-host-tests'
New-Item -ItemType Directory -Path $build -Force | Out-Null
if (-not (Test-Path -LiteralPath $Compiler)) { throw "Host compiler unavailable: $Compiler" }
# The installed Windows Clang can emit Linux objects. Existing WSL supplies ld
# and libc, so these checks need no compiler, core or dependency installation.
& $Compiler --target=x86_64-unknown-linux-gnu -std=c++14 -O1 -Wall -Wextra -Werror `
    -fno-exceptions -fno-rtti -fno-stack-protector -nostdinc `
    -isystem (Join-Path $fixture 'host_c') -I $fixture `
    -DARDUINO_UNOR4_WIFI -DNO_USB -c (Join-Path $fixture 'sensor_tests.cpp') `
    -o (Join-Path $build 'sensor_tests.o')
if ($LASTEXITCODE -ne 0) { throw 'Sensor host compilation failed.' }
& $Compiler --target=x86_64-unknown-linux-gnu -c (Join-Path $fixture 'linux_entry.S') `
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
# Exercise the unmodified Python parser against rows emitted by actual C++
# production formatting, including bounds/invalid rows and ignored motor lines.
# This is a test invocation; no Python source or Python test file is changed.
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
print("Existing Python parse_line/LineDecoder: 8 C++ rows accepted; mixed motor lines ignored.")
'@
& $python -c $parserCheck (Join-Path $projectRoot 'code') $rowsPath
if ($LASTEXITCODE -ne 0) { throw 'Existing telemetry parser compatibility failed.' }
Write-Output 'Sensor logic, actual-sketch simulation and CSV compatibility passed. No hardware accessed.'
