# SPDX-License-Identifier: GPL-3.0-or-later
# Builds (if anything changed) and starts the OpenVRX live app on Windows.
# Usage:  .\tools\vrx.ps1 [app arguments]     (for example --help)
#         -NoBuild skips the cargo build and starts the last build.
#
# stderr goes to %LOCALAPPDATA%\openvrx\vrx-<time>.log (the last 20 are kept),
# as `vrx` does on Linux with ~/.local/state/openvrx. To have a `vrx` command
# in every PowerShell window, add this line to $PROFILE.CurrentUserAllHosts:
#   function vrx { & "<repo>\tools\vrx.ps1" @args }

param([switch]$NoBuild)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$hostDir = Join-Path $repo 'host'
$exe = Join-Path $hostDir 'target\release\openvrx.exe'

if (-not $NoBuild) {
    . (Join-Path $repo 'tools\env.ps1') | Out-Null
    Push-Location $hostDir
    try {
        cargo build --release -p openvrx
        if ($LASTEXITCODE -ne 0) { Write-Error "build failed ($LASTEXITCODE)" }
    } finally {
        Pop-Location
    }
}
if (-not (Test-Path $exe)) { Write-Error "$exe not found; run without -NoBuild" }

$logDir = Join-Path $env:LOCALAPPDATA 'openvrx'
New-Item -ItemType Directory -Force $logDir | Out-Null
$log = Join-Path $logDir ("vrx-{0}.log" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
Get-ChildItem $logDir -Filter 'vrx-*.log' | Sort-Object Name -Descending |
    Select-Object -Skip 19 | Remove-Item -Force

$p = @{ FilePath = $exe; WorkingDirectory = $repo; RedirectStandardError = $log; NoNewWindow = $true; PassThru = $true }
if ($args.Count -gt 0) { $p.ArgumentList = $args }
$proc = Start-Process @p
Write-Host "OpenVRX started (pid $($proc.Id)), log $log"
