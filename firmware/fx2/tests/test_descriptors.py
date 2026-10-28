# SPDX-License-Identifier: GPL-3.0-or-later
"""Parses the USB descriptors out of the compiled firmware image and checks
every length field, cross-reference and fixed value.

The descriptor tables are located through the linker map
(build/openvrx-fx2.map) and read from build/openvrx-fx2.ihx, so what is checked
is exactly what the FX2 will send. Run `python build.py --test`.
"""

import re
import struct
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / "tools"))
import fx2image  # noqa: E402

IHX = HERE / "build" / "openvrx-fx2.ihx"
MAP = HERE / "build" / "openvrx-fx2.map"

ORDER = ["device", "qualifier", "config_hs", "config_fs", "lang", "manufacturer", "product",
         "serial_template", "msos10_string", "bos", "msos20_set", "msos10_compat",
         "msos10_props"]

GUID = "{5C2B2B1E-7D5A-4C1B-9E0B-50435652580A}"
MS_OS_20_UUID = bytes.fromhex("DF60DDD88945C74C9CD2659D9E648A9F")
VENDOR_CODE = 0xEE


def utf16(s):
    return s.encode("utf-16-le")


def load():
    image = fx2image.flatten(fx2image.read_ihx(IHX))
    symbols = {}
    for line in MAP.read_text(errors="replace").splitlines():
        m = re.match(r"^\s*C:\s+([0-9A-F]{8})\s+(_openvrx_desc_\w+)", line)
        if m:
            symbols[m.group(2)[len("_openvrx_desc_"):]] = int(m.group(1), 16)
    sizes_at = symbols["sizes"]
    sizes = struct.unpack_from("<13H", image, sizes_at)
    desc = {}
    for name, size in zip(ORDER, sizes):
        addr = symbols[name]
        desc[name] = bytes(image[addr:addr + size])
    return desc


class Descriptors(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not IHX.exists() or not MAP.exists():
            raise unittest.SkipTest("build the firmware first (python build.py)")
        cls.d = load()

    def test_device(self):
        d = self.d["device"]
        self.assertEqual(len(d), 18)
        (bLength, bType, bcdUSB, cls_, sub, proto, mps0, vid, pid, bcdDevice,
         iMan, iProd, iSer, nConf) = struct.unpack("<BBHBBBBHHHBBBB", d)
        self.assertEqual((bLength, bType), (18, 1))
        self.assertEqual(bcdUSB, 0x0201)
        self.assertEqual((cls_, sub, proto), (0xFF, 0xFF, 0xFF))
        self.assertEqual(mps0, 64)
        self.assertEqual((vid, pid), (0x1209, 0x0001))
        self.assertEqual((iMan, iProd, iSer, nConf), (1, 2, 3, 1))

    def test_qualifier(self):
        q = self.d["qualifier"]
        self.assertEqual(len(q), 10)
        self.assertEqual(q[0], 10)
        self.assertEqual(q[1], 6)
        self.assertEqual(struct.unpack_from("<H", q, 2)[0], 0x0200)
        self.assertEqual(q[7], 64)
        self.assertEqual(q[8], 1)
        self.assertEqual(q[9], 0)

    def check_config(self, c, ep2_size, ep1_interval):
        self.assertEqual(c[0], 9)
        self.assertEqual(c[1], 2)
        total = struct.unpack_from("<H", c, 2)[0]
        self.assertEqual(total, len(c))
        self.assertEqual(c[4], 1)                 # bNumInterfaces
        self.assertEqual(c[5], 1)                 # bConfigurationValue
        self.assertTrue(c[7] & 0x80)              # reserved bit set
        self.assertLessEqual(c[8], 250)           # bMaxPower, 2 mA units
        pos, endpoints, interfaces, declared_eps = 9, {}, 0, None
        while pos < total:
            length, dtype = c[pos], c[pos + 1]
            self.assertGreaterEqual(length, 2)
            self.assertLessEqual(pos + length, total)
            if dtype == 4:
                self.assertEqual(length, 9)
                interfaces += 1
                declared_eps = c[pos + 4]
                self.assertEqual(c[pos + 2], 0)
                self.assertEqual(c[pos + 3], 0)
            elif dtype == 5:
                self.assertEqual(length, 7)
                addr, attr, mps, interval = struct.unpack_from("<BBHB", c, pos + 2)
                endpoints[addr] = (attr, mps, interval)
            else:
                self.fail(f"unexpected descriptor type {dtype}")
            pos += length
        self.assertEqual(pos, total)
        self.assertEqual(interfaces, 1)
        self.assertEqual(declared_eps, len(endpoints))
        self.assertEqual(endpoints[0x82], (2, ep2_size, 0))
        self.assertEqual(endpoints[0x81], (3, 64, ep1_interval))

    def test_config_high_speed(self):
        self.check_config(self.d["config_hs"], 512, 7)   # 2^(7-1) x 125 us = 8 ms

    def test_config_full_speed(self):
        self.check_config(self.d["config_fs"], 64, 8)    # 8 frames = 8 ms

    def test_other_speed_is_same_layout(self):
        hs, fs = self.d["config_hs"], self.d["config_fs"]
        self.assertEqual(len(hs), len(fs))

    def check_string(self, s, text):
        self.assertEqual(s[0], len(s))
        self.assertEqual(s[1], 3)
        self.assertEqual(s[2:], utf16(text))

    def test_strings(self):
        self.assertEqual(self.d["lang"], bytes([4, 3, 0x09, 0x04]))
        self.check_string(self.d["manufacturer"], "OpenVRX")
        self.check_string(self.d["product"], "OpenVRX FX2")
        self.check_string(self.d["serial_template"], "00000000")
        self.assertEqual(len(self.d["serial_template"]), 18)

    def test_msos10_string(self):
        s = self.d["msos10_string"]
        self.assertEqual(len(s), 18)
        self.assertEqual(s[0], 18)
        self.assertEqual(s[1], 3)
        self.assertEqual(s[2:16], utf16("MSFT100"))
        self.assertEqual(s[16], VENDOR_CODE)
        self.assertEqual(s[17], 0)

    def test_bos(self):
        b = self.d["bos"]
        self.assertEqual(b[0], 5)
        self.assertEqual(b[1], 0x0F)
        total, ncaps = struct.unpack_from("<HB", b, 2)
        self.assertEqual(total, len(b))
        pos, caps = 5, []
        while pos < total:
            length, dtype, captype = b[pos], b[pos + 1], b[pos + 2]
            self.assertEqual(dtype, 0x10)
            self.assertLessEqual(pos + length, total)
            caps.append((captype, b[pos:pos + length]))
            pos += length
        self.assertEqual(pos, total)
        self.assertEqual(len(caps), ncaps)
        usb2 = [c for t, c in caps if t == 0x02]
        self.assertEqual(len(usb2), 1)
        self.assertEqual(len(usb2[0]), 7)
        self.assertEqual(struct.unpack_from("<I", usb2[0], 3)[0], 0)   # no LPM
        plat = [c for t, c in caps if t == 0x05]
        self.assertEqual(len(plat), 1)
        p = plat[0]
        self.assertEqual(len(p), 28)
        self.assertEqual(p[3], 0)
        self.assertEqual(p[4:20], MS_OS_20_UUID)
        winver, setlen, vendor, altenum = struct.unpack_from("<IHBB", p, 20)
        self.assertEqual(winver, 0x06030000)
        self.assertEqual(setlen, len(self.d["msos20_set"]))
        self.assertEqual(vendor, VENDOR_CODE)
        self.assertEqual(altenum, 0)

    def test_msos20_set(self):
        s = self.d["msos20_set"]
        length, dtype, winver, total = struct.unpack_from("<HHIH", s, 0)
        self.assertEqual((length, dtype, winver), (10, 0x0000, 0x06030000))
        self.assertEqual(total, len(s))
        pos, found = 10, {}
        while pos < total:
            length, dtype = struct.unpack_from("<HH", s, pos)
            self.assertGreaterEqual(length, 4)
            self.assertLessEqual(pos + length, total)
            found[dtype] = s[pos:pos + length]
            pos += length
        self.assertEqual(pos, total)
        cid = found[0x0003]
        self.assertEqual(len(cid), 20)
        self.assertEqual(cid[4:12], b"WINUSB\0\0")
        self.assertEqual(cid[12:20], bytes(8))
        reg = found[0x0004]
        ptype, nlen = struct.unpack_from("<HH", reg, 4)
        self.assertEqual(ptype, 7)                       # REG_MULTI_SZ
        name = reg[8:8 + nlen]
        self.assertEqual(name, utf16("DeviceInterfaceGUIDs\0"))
        dlen = struct.unpack_from("<H", reg, 8 + nlen)[0]
        data = reg[10 + nlen:10 + nlen + dlen]
        self.assertEqual(data, utf16(GUID + "\0\0"))
        self.assertEqual(len(reg), 10 + nlen + dlen)

    def test_msos10_compat(self):
        c = self.d["msos10_compat"]
        dwlen, bcd, windex, count = struct.unpack_from("<IHHB", c, 0)
        self.assertEqual(dwlen, len(c))
        self.assertEqual((bcd, windex), (0x0100, 0x0004))
        self.assertEqual(c[9:16], bytes(7))
        self.assertEqual(16 + 24 * count, len(c))
        f = c[16:40]
        self.assertEqual(f[0], 0)                        # interface 0
        self.assertEqual(f[1], 1)                        # reserved, must be 1
        self.assertEqual(f[2:10], b"WINUSB\0\0")
        self.assertEqual(f[10:18], bytes(8))
        self.assertEqual(f[18:24], bytes(6))

    def test_msos10_props(self):
        p = self.d["msos10_props"]
        dwlen, bcd, windex, count = struct.unpack_from("<IHHH", p, 0)
        self.assertEqual(dwlen, len(p))
        self.assertEqual((bcd, windex, count), (0x0100, 0x0005, 1))
        size, ptype, nlen = struct.unpack_from("<IIH", p, 10)
        self.assertEqual(10 + size, len(p))
        self.assertEqual(ptype, 7)
        name = p[20:20 + nlen]
        self.assertEqual(name, utf16("DeviceInterfaceGUIDs\0"))
        dlen = struct.unpack_from("<I", p, 20 + nlen)[0]
        data = p[24 + nlen:24 + nlen + dlen]
        self.assertEqual(data, utf16(GUID + "\0\0"))
        self.assertEqual(size, 4 + 4 + 2 + nlen + 4 + dlen)


if __name__ == "__main__":
    unittest.main()
