@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "SO101_PYTHON=%USERPROFILE%\miniconda3\python.exe"
set "SO101_HAND_MODEL=C:\Users\Lenovo\Desktop\SO101_3_modes\ground\hand_landmarker.task"
if not exist "%SO101_PYTHON%" set "SO101_PYTHON=python"
"%SO101_PYTHON%" -m pip install -q fastapi uvicorn
if not exist "web\node_modules" call npm --prefix web install
if not exist "web\dist\index.html" call npm --prefix web run build
"%SO101_PYTHON%" web_server.py
if errorlevel 1 pause
