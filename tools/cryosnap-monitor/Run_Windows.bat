@echo off
REM Double-click this file to start the CryoSnap Monitor.
REM It runs the PowerShell launcher with script execution allowed for this run only.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0monitor_cryosnap.ps1"
