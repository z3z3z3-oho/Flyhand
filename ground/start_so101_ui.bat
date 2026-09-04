@echo off
chcp 65001 >nul
cd /d "%~dp0"
python ground_so101_ui.py
if errorlevel 1 pause
