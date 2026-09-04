@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "SO101_PYTHON=%USERPROFILE%\miniconda3\python.exe"
set "SO101_HAND_MODEL=C:\Users\Lenovo\Desktop\SO101_3_modes\ground\hand_landmarker.task"
set "SO101_UI_DIR=so101-control-ui"
if not exist "%SO101_PYTHON%" set "SO101_PYTHON=python"
"%SO101_PYTHON%" -c "import fastapi, uvicorn" >nul 2>nul
if errorlevel 1 "%SO101_PYTHON%" -m pip install -q fastapi uvicorn
if not exist "%SO101_UI_DIR%\package.json" (
  echo Missing UI project: %SO101_UI_DIR%
  pause
  exit /b 1
)
if not exist "%SO101_UI_DIR%\node_modules" call npm --prefix "%SO101_UI_DIR%" install
if not exist "%SO101_UI_DIR%\dist\index.html" call npm --prefix "%SO101_UI_DIR%" run build
"%SO101_PYTHON%" web_server.py
if errorlevel 1 pause
