// SPDX-License-Identifier: GPL-3.0-or-later
//! The OpenVRX FX2 firmware image: embedded at build time when
//! `firmware/fx2/prebuilt/openvrx-fx2.ihx` exists, or loaded from a file.

use crate::error::{HwError, Result};
use crate::fx2::image::{FirmwareImage, IicHeader, IicImage};
use crate::fx2::protocol::{OPENVRX_PID, OPENVRX_VID};
use std::path::Path;

static EMBEDDED: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/openvrx-fx2.ihx"));

/// True when an image was embedded at build time.
pub fn has_embedded() -> bool {
    !EMBEDDED.is_empty()
}

/// The embedded image, or a clear error when the crate was built before
/// the firmware existed.
pub fn embedded() -> Result<FirmwareImage> {
    if EMBEDDED.is_empty() {
        return Err(HwError::NoFirmware(
            "firmware/fx2/prebuilt/openvrx-fx2.ihx did not exist when openvrx-hw was built; \
             build the FX2 firmware and rebuild, or pass the .ihx path explicitly"
                .into(),
        ));
    }
    let text = std::str::from_utf8(EMBEDDED)
        .map_err(|_| HwError::NoFirmware("embedded firmware is not Intel HEX text".into()))?;
    let img = FirmwareImage::parse_ihex(text)?;
    img.check_fx2_ram()?;
    Ok(img)
}

/// Loads an Intel HEX file and checks it fits the FX2 RAM.
pub fn load_ihex(path: &Path) -> Result<FirmwareImage> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| HwError::NoFirmware(format!("cannot read {}: {e}", path.display())))?;
    let img = FirmwareImage::parse_ihex(&text)?;
    img.check_fx2_ram()?;
    Ok(img)
}

/// The image from `path` when given, else the embedded one.
pub fn resolve(path: Option<&Path>) -> Result<FirmwareImage> {
    match path {
        Some(p) => load_ihex(p),
        None => embedded(),
    }
}

/// Loads and validates a C2 `.iic` EEPROM image.
pub fn load_iic(path: &Path) -> Result<Vec<u8>> {
    let bytes = std::fs::read(path)?;
    IicImage::parse(&bytes)?;
    Ok(bytes)
}

/// Packs a RAM image into a C2 EEPROM image with the OpenVRX IDs.
pub fn iic_for(fw: &FirmwareImage) -> Result<Vec<u8>> {
    IicImage::build(
        IicHeader {
            vid: OPENVRX_VID,
            pid: OPENVRX_PID,
            did: 0x0001,
            config: 0x00,
        },
        fw,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_matches_availability() {
        match embedded() {
            Ok(img) => {
                assert!(has_embedded());
                assert!(!img.is_empty());
                let iic = iic_for(&img).unwrap();
                assert_eq!(IicImage::parse(&iic).unwrap().firmware, img);
            }
            Err(HwError::NoFirmware(msg)) => {
                assert!(!has_embedded());
                assert!(msg.contains("openvrx-fx2.ihx"));
            }
            Err(e) => panic!("embedded firmware is invalid: {e}"),
        }
    }

    #[test]
    fn load_from_file() {
        let dir = std::env::temp_dir().join(format!("openvrx-hw-fw-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("t.ihx");
        std::fs::write(&p, ":0300000002000CEF\n:00000001FF\n").unwrap();
        let img = resolve(Some(&p)).unwrap();
        assert_eq!(img.len(), 3);
        assert!(load_ihex(&dir.join("missing.ihx")).is_err());
        let iic = dir.join("t.iic");
        std::fs::write(&iic, iic_for(&img).unwrap()).unwrap();
        assert!(load_iic(&iic).is_ok());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
