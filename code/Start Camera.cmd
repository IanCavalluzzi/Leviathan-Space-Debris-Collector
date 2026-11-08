@echo off
setlocal
cd /d "%~dp0"
if not exist "..\.venv\Scripts\python.exe" (
    echo The project Python environment is missing. Open the project README.md for setup instructions.
    pause
    exit /b 1
)
"..\.venv\Scripts\python.exe" "camera_preview.py" %*
set "preview_exit=%errorlevel%"
if not "%preview_exit%"=="0" pause
exit /b %preview_exit%
