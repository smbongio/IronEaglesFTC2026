@echo off
rem Runs pulllogs.ps1 without changing the system's PowerShell execution policy.
rem Pulls logs then rotates the robot log. Use -KeepRemote to leave the robot log intact.
rem Example: pulllogs -Address 192.168.43.1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pulllogs.ps1" %*
exit /b %ERRORLEVEL%
