# SPDX-License-Identifier: GPL-3.0-or-later
"""Checks the firmware images: layout limits, the prebuilt copy, and that
the C2 EEPROM image loads exactly the bytes of the Intel HEX image."""

import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / "tools"))
import fx2image  # noqa: E402

BUILD_IHX = HERE / "build" / "openvrx-fx2.ihx"
PREBUILT_IHX = HERE / "prebuilt" / "openvrx-fx2.ihx"
PREBUILT_IIC = HERE / "prebuilt" / "openvrx-fx2.iic"


class Images(unittest.TestCase):
    def test_prebuilt_layout(self):
        segs = fx2image.read_ihx(PREBUILT_IHX)
        end = max(a + len(d) for a, d in segs)
        self.assertLessEqual(end, 0x3800)              # code area
        self.assertEqual(segs[0][0], 0x0000)           # reset vector present
        self.assertEqual(segs[0][1][0], 0x02)          # LJMP at the reset vector

    def test_prebuilt_matches_build(self):
        if not BUILD_IHX.exists():
            self.skipTest("no local build")
        self.assertEqual(BUILD_IHX.read_bytes(), PREBUILT_IHX.read_bytes())

    def test_iic_roundtrip(self):
        segs = fx2image.read_ihx(PREBUILT_IHX)
        vid, pid, did, config, loaded = fx2image.parse_c2(PREBUILT_IIC.read_bytes())
        self.assertEqual((vid, pid), (0x1209, 0x0001))
        self.assertEqual(config, 0x00)
        self.assertEqual(fx2image.flatten(loaded), fx2image.flatten(segs))
        # The image must leave the EEPROM board ID record (0x3FF8) free.
        self.assertLess(len(PREBUILT_IIC.read_bytes()), 0x3FF8)

    def test_c2_codec(self):
        segs = [(0x0000, bytes(range(256)) * 5), (0x2000, b"\x01\x02\x03")]
        img = fx2image.make_c2(segs, 0x1209, 0x0001, 0x0010)
        _, _, _, _, out = fx2image.parse_c2(img)
        self.assertTrue(all(len(d) <= 1023 for _, d in out))
        self.assertEqual(fx2image.flatten(out), fx2image.flatten(segs))


if __name__ == "__main__":
    unittest.main()
