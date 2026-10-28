# SPDX-License-Identifier: GPL-3.0-or-later
"""Checks the cycle annotations of src/hotloops.asm against its listing.

Reads the sdas8051 listing (build/obj/hotloops.lst), looks up the FX2
instruction cycles of every assembled opcode (TRM table 12-2, MOVX with
stretch 0), compares them with the "; N" comment of the source line where
one is present, and prints the cycle sum of every labelled block.
Exits non-zero on an unknown opcode or a wrong annotation.
"""

import re
import sys

# opcode -> FX2 instruction cycles (only opcodes used by hotloops.asm need
# to be listed; anything else is reported).
FX2_CYCLES = {
    0x05: 2,  # inc direct
    0x08: 1,  # inc r0
    0x0E: 1, 0x0F: 1,  # inc r6 / r7
    0x13: 1,  # rrc a
    0x15: 2,  # dec direct
    0x20: 4,  # jb bit, rel
    0x22: 4,  # ret
    0x30: 4,  # jnb bit, rel
    0x40: 3,  # jc
    0x45: 2,  # orl a, direct
    0x50: 3,  # jnc
    0x54: 2,  # anl a, #imm
    0x60: 3,  # jz
    0x70: 3,  # jnz
    0x74: 2,  # mov a, #imm
    0x75: 3,  # mov direct, #imm
    0x78: 2,  # mov r0, #imm
    0x7D: 2,  # mov r5, #imm
    0x80: 3,  # sjmp
    0x85: 3,  # mov direct, direct
    0x8E: 2, 0x8F: 2,  # mov direct, r6 / r7
    0x90: 3,  # mov dptr, #imm16
    0x9F: 1,  # subb a, r7
    0xAD: 2, 0xAE: 2, 0xAF: 2,  # mov r5/r6/r7, direct
    0xC3: 1,  # clr c
    0xC4: 1,  # swap a
    0xDD: 3,  # djnz r5, rel
    0xE0: 2,  # movx a, @dptr
    0xE4: 1,  # clr a
    0xE5: 2,  # mov a, direct
    0xEE: 1, 0xEF: 1,  # mov a, r6 / r7
    0xF0: 2,  # movx @dptr, a
    0xF2: 2,  # movx @r0, a
    0xFF: 1,  # mov r7, a
}

LINE = re.compile(r"^\s+([0-9A-F]{6})\s((?:[0-9A-F]{2}[ *]?)+)\s+\[\s*\d+\]\s+\d+\s(.*)$")
LABEL = re.compile(r"^\s+[0-9A-F]{6}\s+\d+\s+(\w+):")


def main(path):
    errors = 0
    blocks = []
    with open(path, encoding="ascii", errors="replace") as f:
        for raw in f:
            m = LABEL.match(raw)
            if m:
                blocks.append([m.group(1), 0, 0])
                continue
            m = LINE.match(raw)
            if not m:
                continue
            opcode = int(m.group(2)[:2], 16)
            src = m.group(3)
            if opcode not in FX2_CYCLES:
                print(f"unknown opcode 0x{opcode:02X}: {src.strip()}")
                errors += 1
                continue
            cyc = FX2_CYCLES[opcode]
            note = re.search(r";\s*(\d+)\b", src)
            if note and int(note.group(1)) != cyc:
                print(f"annotation {note.group(1)} != {cyc} cycles: {src.strip()}")
                errors += 1
            if blocks:
                blocks[-1][1] += cyc
                blocks[-1][2] += 1
    print("hot loop blocks (straight-line FX2 cycles, instructions):")
    for name, cyc, n in blocks:
        print(f"  {name:16s} {cyc:5d} cycles  {n:4d} instructions")
    if errors:
        print(f"{errors} error(s)")
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv[1])
