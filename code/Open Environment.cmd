@echo off
setlocal
cd /d "%~dp0"
if not exist "..\.venv\Scripts\Activate.ps1" (
    echo The project Python environment is missing. Open the project README.md for setup instructions.
    pause
    exit /b 1
)
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -NoExit -ExecutionPolicy Bypass -Command ". '..\.venv\Scripts\Activate.ps1'; Write-Host 'Debris Capture environment ready in code/.'; Write-Host 'Start video: python .\camera_preview.py --camera 1 --backend dshow'; Write-Host 'List USB serial ports: python .\camera_preview.py --list-ports'"
