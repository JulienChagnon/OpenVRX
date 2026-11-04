// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared helpers for the integration tests.
#![allow(dead_code)]

use openvrx_core::{Field, FieldParity, LineRecord, VideoStandard, YuvField};
use openvrx_video::encoder::{CompositeEncoder, EncoderConfig};
use openvrx_video::testgen::{field_info, image_to_field, YuvImage};

/// Assembles records of one field into a `Field`.
pub fn to_field(std: VideoStandard, recs: &[LineRecord]) -> Field {
    let mut f = Field::new(std, recs[0].field_seq, recs[0].parity);
    for r in recs {
        f.put_line(r);
    }
    f
}

/// Encodes field `seq` of a picture sequence (picture shown at that field).
pub fn encode(enc: &CompositeEncoder, pic: &YuvImage, seq: u64) -> Vec<LineRecord> {
    let std = enc.params().standard;
    let (par, fi) = field_info(seq);
    let f = image_to_field(pic, std, par, seq);
    let field_ns = 1e9 / enc.params().field_hz;
    enc.encode_field(&f, fi, (seq as f64 * field_ns) as u64 + 1)
}

pub fn encoder(std: VideoStandard) -> CompositeEncoder {
    CompositeEncoder::new(std, EncoderConfig::default())
}

/// The truth for a decoded field: the source picture's rows of that field.
pub fn truth_field(pic: &YuvImage, std: VideoStandard, seq: u64) -> YuvField {
    let (par, _) = field_info(seq);
    image_to_field(pic, std, par, seq)
}

pub fn parity(seq: u64) -> FieldParity {
    field_info(seq).0
}
