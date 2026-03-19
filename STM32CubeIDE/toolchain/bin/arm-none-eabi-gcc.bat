@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0arm-none-eabi-gcc.ps1" %*
exit /b %errorlevel%
