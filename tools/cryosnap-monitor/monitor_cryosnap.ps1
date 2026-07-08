# CryoSnap Monitor Launcher
# Right-click -> Run with PowerShell

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pyScript  = Join-Path $scriptDir "cryosnap_monitor.py"

# The monitor opens a single window on the second monitor with the serial terminal
# embedded on the left 22% and the live graphs on the right 78%. This console is
# hidden while the monitor runs and reappears when it closes.

# ── Find Python 3 ─────────────────────────────────────────────────────────────
$python = $null
foreach ($cmd in @("python", "python3", "py")) {
    try {
        $ver = & $cmd --version 2>&1
        if ($ver -match "Python 3") {
            $python = $cmd
            break
        }
    } catch {}
}

if (-not $python) {
    Write-Host ""
    Write-Host "Python 3 not found." -ForegroundColor Red
    Write-Host "Download and install it from: https://www.python.org/downloads/" -ForegroundColor Yellow
    Write-Host "During install, make sure to check 'Add Python to PATH'." -ForegroundColor Yellow
    Write-Host ""
    pause
    exit
}

Write-Host "Python found ($python)." -ForegroundColor Green

# ── Install / update dependencies ─────────────────────────────────────────────
Write-Host "Checking dependencies (pyserial, matplotlib) ..." -ForegroundColor Cyan
& $python -m pip install pyserial matplotlib --quiet --upgrade

if ($LASTEXITCODE -ne 0) {
    Write-Host "pip install failed. Check your internet connection." -ForegroundColor Red
    pause
    exit
}

Write-Host "Dependencies OK." -ForegroundColor Green
Write-Host ""
Write-Host "Starting CryoSnap Monitor ..." -ForegroundColor Cyan
Write-Host "A single window opens on your second monitor (terminal left, graphs right)." -ForegroundColor DarkGray
Write-Host "Type commands in the terminal pane; 'x' + Enter drops and reconnects the serial port." -ForegroundColor DarkGray
Write-Host ""

# ── Run monitor ────────────────────────────────────────────────────────────────
& $python $pyScript

Write-Host ""
Write-Host "Monitor closed." -ForegroundColor Yellow
pause
