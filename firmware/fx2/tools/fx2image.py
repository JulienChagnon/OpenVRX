# SPDX-License-Identifier: GPL-3.0-or-later
"""Intel HEX and FX2 boot EEPROM (C2 load) image helpers.

C2 format (EZ-USB TRM 3.4.3): 0xC2, VID, PID, DID (little endian), one
configuration byte, then records of [length (big endian, 10 bits)]
[address (big endian)][data], closed by the record 0x8001 0xE600 0x00 that
writes 0 to CPUCS and releases the 8051 from reset.
"""

import struct

C2_MAX_RECORD = 1023
CPUCS = 0xE600


def read_ihx(path):
    """Returns a sorted list of (address, bytes) with contiguous runs merged."""
    mem = {}
    with open(path, "r", encoding="ascii") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"{path}:{lineno}: not an Intel HEX record")
            raw = bytes.fromhex(line[1:])
            if len(raw) < 5 or len(raw) != raw[0] + 5:
                raise ValueError(f"{path}:{lineno}: bad record length")
            if sum(raw) & 0xFF:
                raise ValueError(f"{path}:{lineno}: bad checksum")
            count, addr, rectype = raw[0], (raw[1] << 8) | raw[2], raw[3]
            if rectype == 0x00:
                for i, b in enumerate(raw[4:4 + count]):
                    if addr + i in mem and mem[addr + i] != b:
                        raise ValueError(f"{path}:{lineno}: overlapping data")
                    mem[addr + i] = b
            elif rectype == 0x01:
                break
            else:
                raise ValueError(f"{path}:{lineno}: unsupported record type {rectype}")
    segments = []
    for addr in sorted(mem):
        if segments and segments[-1][0] + len(segments[-1][1]) == addr:
            segments[-1][1].append(mem[addr])
        else:
            segments.append((addr, bytearray([mem[addr]])))
    return [(a, bytes(d)) for a, d in segments]


def flatten(segments, size=0x4000, fill=0x00):
    image = bytearray([fill]) * size
    for addr, data in segments:
        if addr + len(data) > size:
            raise ValueError(f"data at 0x{addr:04X} exceeds 0x{size:04X}")
        image[addr:addr + len(data)] = data
    return image


def make_c2(segments, vid, pid, did, config=0x00):
    """Builds a C2 boot image. config bit 0 = 400 kHz I2C, bit 6 = DISCON."""
    out = bytearray([0xC2])
    out += struct.pack("<HHH", vid, pid, did)
    out.append(config)
    for addr, data in segments:
        if not (addr + len(data) <= 0x4000 or (addr >= 0xE000 and addr + len(data) <= 0xE200)):
            raise ValueError(f"segment 0x{addr:04X}+{len(data)} is not loadable from EEPROM")
        pos = 0
        while pos < len(data):
            chunk = data[pos:pos + C2_MAX_RECORD]
            out += struct.pack(">HH", len(chunk), addr + pos)
            out += chunk
            pos += len(chunk)
    out += struct.pack(">HHB", 0x8001, CPUCS, 0x00)
    return bytes(out)


def parse_c2(image):
    """Inverse of make_c2: returns (vid, pid, did, config, segments)."""
    if image[0] != 0xC2:
        raise ValueError("not a C2 image")
    vid, pid, did, config = struct.unpack_from("<HHHB", image, 1)
    pos = 8
    segments = []
    while True:
        if pos + 4 > len(image):
            raise ValueError("truncated record header")
        length, addr = struct.unpack_from(">HH", image, pos)
        pos += 4
        if length & 0x8000:
            if (length & 0x3FF) != 1 or addr != CPUCS or image[pos] != 0x00:
                raise ValueError("bad final record")
            pos += 1
            break
        if length == 0 or length > C2_MAX_RECORD or pos + length > len(image):
            raise ValueError("bad data record")
        segments.append((addr, bytes(image[pos:pos + length])))
        pos += length
    if pos != len(image):
        raise ValueError("trailing bytes after the final record")
    return vid, pid, did, config, segments


def write_ihx(path, segments):
    with open(path, "w", encoding="ascii", newline="\n") as f:
        for addr, data in segments:
            for pos in range(0, len(data), 16):
                chunk = data[pos:pos + 16]
                a = addr + pos
                rec = bytes([len(chunk), a >> 8, a & 0xFF, 0x00]) + chunk
                f.write(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper() + "\n")
        f.write(":00000001FF\n")
