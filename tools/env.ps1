# SPDX-License-Identifier: GPL-3.0-or-later
# Sets up a PowerShell session for building OpenVRX on Windows.
# Usage:  . .\tools\env.ps1
#
# The host is built with Rust's x86_64-pc-windows-gnu toolchain. Crates that
# use raw-dylib imports need a 64-bit dlltool on PATH ahead of any 32-bit
# MinGW install, so a 64-bit MinGW-w64 directory is put first.

$candidates = @(
    "C:\ProgramData\mingw64\mingw64\bin",
    "C:\msys64\mingw64\bin",
    "$env:USERPROFILE\.rustup\toolchains\stable-x86_64-pc-windows-gnu\lib\rustlib\x86_64-pc-windows-gnu\bin\self-contained"
)
$mingw = $candidates | Where-Object { Test-Path (Join-Path $_ "dlltool.exe") } | Select-Object -First 1
if ($null -eq $mingw) {
    Write-Warning "No 64-bit dlltool found; install MinGW-w64 or the Rust GNU toolchain."
} else {
    $env:Path = "$mingw;$env:USERPROFILE\.cargo\bin;$env:Path"
    Write-Host "OpenVRX: using $mingw for dlltool/gcc"
}

$sdcc = "C:\Program Files\SDCC\bin"
if (Test-Path $sdcc) { $env:Path = "$env:Path;$sdcc" }

$idfProfile = "C:\Espressif\tools\Microsoft.v6.0.2.PowerShell_profile.ps1"
if (Test-Path $idfProfile) {
    Write-Host "OpenVRX: ESP-IDF available; run '. $idfProfile' to activate it for firmware/c5."
}
