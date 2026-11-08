@echo off
setlocal
cd /d "%~dp0"
if not exist "..\.venv\Scripts\python.exe" (
    echo The project Python environment is missing. See README.md for setup.
    pause
    exit /b 1
)
"..\.venv\Scripts\python.exe" "manual_motors.py" %*
set "manual_exit=%errorlevel%"
if not "%manual_exit%"=="0" pause
exit /b %manual_exit%
