@echo off
rem Runs pushconfig.ps1 without changing the system's PowerShell execution policy. Arguments pass through, e.g. pushconfig -Address 192.168.43.1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pushconfig.ps1" %*
exit /b %ERRORLEVEL%
