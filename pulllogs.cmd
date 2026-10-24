@echo off
rem Runs pulllogs.ps1 without changing the system's PowerShell execution policy. Arguments pass through, e.g. pulllogs -Address 192.168.43.1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pulllogs.ps1" %*
exit /b %ERRORLEVEL%
