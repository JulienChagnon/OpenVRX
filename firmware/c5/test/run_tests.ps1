# SPDX-License-Identifier: GPL-3.0-or-later
# Builds and runs the host unit tests of the C5 radio head pure logic.
# Uses $env:CC if set, otherwise the first gcc/clang found on PATH or in
# the usual MinGW locations. Extra arguments go to the test program
# (for example --tables).
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$logic = Join-Path $here '..\main\logic'
$cc = $env:CC
if (-not $cc) {
    foreach ($c in @('gcc', 'clang', 'C:\ProgramData\mingw64\mingw64\bin\gcc.exe', 'C:\msys64\ucrt64\bin\gcc.exe', 'C:\MinGW\bin\gcc.exe')) {
        if (Get-Command $c -ErrorAction SilentlyContinue) { $cc = $c; break }
    }
}
if (-not $cc) { Write-Error 'no C compiler found (set CC)'; exit 2 }
$out = Join-Path ([System.IO.Path]::GetTempPath()) 'openvrx_c5_tests.exe'
$srcs = @(Join-Path $here 'test_logic.c') + (Get-ChildItem (Join-Path $logic '*.c') | ForEach-Object { $_.FullName })
& $cc -std=c11 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion "-I$logic" @srcs -o $out
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $out @args
exit $LASTEXITCODE
