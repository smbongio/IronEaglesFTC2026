@echo off
rem Runs deploy.ps1 without changing the system's PowerShell execution policy. Arguments pass through, e.g. deploy -Address 192.168.43.1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" %*
exit /b %ERRORLEVEL%
