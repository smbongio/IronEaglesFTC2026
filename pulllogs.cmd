@echo off
rem Runs pulllogs.ps1 without changing the system's PowerShell execution policy.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pulllogs.ps1" %*
exit /b %ERRORLEVEL%
