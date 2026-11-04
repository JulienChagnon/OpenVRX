#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Builds and runs the host unit tests of the C5 radio head pure logic.
# Uses $CC if set, otherwise the first of gcc, cc, clang found on PATH.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
logic="$here/../main/logic"
out="${TMPDIR:-/tmp}/openvrx_c5_tests"
cc="${CC:-}"
if [ -z "$cc" ]; then
    for c in gcc cc clang; do
        if command -v "$c" >/dev/null 2>&1; then cc="$c"; break; fi
    done
fi
if [ -z "$cc" ]; then
    echo "no C compiler found (set CC)" >&2
    exit 2
fi
"$cc" -std=c11 -O2 -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion \
    -I"$logic" "$here/test_logic.c" "$logic"/*.c -lm -o "$out"
"$out" "$@"
