# SPDX-License-Identifier: GPL-3.0-or-later
# Sets up a Git Bash / MSYS shell for building OpenVRX on Windows.
# Usage:  source tools/env.sh
#
# Rust's x86_64-pc-windows-gnu toolchain needs a 64-bit dlltool on PATH ahead
# of any 32-bit MinGW install (raw-dylib imports in the windows crates).

for d in /c/ProgramData/mingw64/mingw64/bin /c/msys64/mingw64/bin \
         "$HOME/.rustup/toolchains/stable-x86_64-pc-windows-gnu/lib/rustlib/x86_64-pc-windows-gnu/bin/self-contained"; do
    if [ -x "$d/dlltool.exe" ]; then
        export PATH="$d:$HOME/.cargo/bin:$PATH"
        echo "OpenVRX: using $d for dlltool/gcc"
        break
    fi
done
if [ -d "/c/Program Files/SDCC/bin" ]; then
    export PATH="$PATH:/c/Program Files/SDCC/bin"
fi
