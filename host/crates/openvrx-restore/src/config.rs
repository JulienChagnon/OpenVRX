// SPDX-License-Identifier: GPL-3.0-or-later
//! Restoration settings and presets.

use crate::flow::FlowParams;
use crate::RestoreError;
use serde::{Deserialize, Serialize};

/// Where pixel confidence comes from.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DamageSource {
    /// The decoder's damage mask and line quality when a frame carries any,
    /// otherwise the blind estimate.
    Auto,
    /// Only the mask and line quality (unflagged pixels are never touched
    /// except by the optional denoise).
    Mask,
    /// Only the blind estimate (ordinary DVR files).
    Blind,
    /// Both, combined with `min`.
    MaskAndBlind,
}

impl std::str::FromStr for DamageSource {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "auto" => Ok(DamageSource::Auto),
            "mask" => Ok(DamageSource::Mask),
            "blind" => Ok(DamageSource::Blind),
            "mask-and-blind" => Ok(DamageSource::MaskAndBlind),
            o => Err(format!("unknown damage source `{o}` (auto|mask|blind|mask-and-blind)")),
        }
    }
}

/// x264 output settings for file restoration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EncodeConfig {
    /// x264 constant rate factor (0 lossless .. 51).
    pub crf: u8,
    /// x264 speed preset (ultrafast .. veryslow).
    pub preset: String,
    /// Start time in seconds within the input.
    pub start: Option<f64>,
    /// Length in seconds to process.
    pub duration: Option<f64>,
}

impl Default for EncodeConfig {
    fn default() -> Self {
        EncodeConfig {
            crf: 16,
            preset: "medium".into(),
            start: None,
            duration: None,
        }
    }
}

/// Everything the restorer needs to know.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RestoreConfig {
    /// Neighbour pictures used on each side (same field parity for fields).
    pub radius: usize,
    /// Motion estimation.
    pub flow: FlowParams,
    /// Check forward/backward flow agreement to reject occluded sources.
    pub consistency: bool,
    /// Motion-compensated temporal denoise of trusted pixels; 0 = off
    /// (trusted pixels stay bit-exact), 1 = nominal.
    pub denoise: f32,
    /// Longest run of broken pictures (in same-parity steps) rebuilt by
    /// interpolating between the good pictures around it.
    pub max_gap: usize,
    /// Pictures whose mean confidence is below this are rebuilt whole.
    pub broken_threshold: f32,
    pub damage_source: DamageSource,
    /// Blind estimate thresholds scale (1 nominal, higher flags more).
    pub blind_sensitivity: f32,
    pub encode: EncodeConfig,
}

impl Default for RestoreConfig {
    fn default() -> Self {
        RestoreConfig::balanced()
    }
}

impl RestoreConfig {
    /// One neighbour each side, no consistency check.
    pub fn fast() -> Self {
        RestoreConfig {
            radius: 1,
            flow: FlowParams {
                block: 12,
                coarse_search: 5,
                refine_search: 1,
                max_levels: 4,
            },
            consistency: false,
            max_gap: 6,
            ..RestoreConfig::balanced()
        }
    }

    /// Two neighbours each side with occlusion checks.
    pub fn balanced() -> Self {
        RestoreConfig {
            radius: 2,
            flow: FlowParams::default(),
            consistency: true,
            denoise: 0.0,
            max_gap: 12,
            broken_threshold: 0.3,
            damage_source: DamageSource::Auto,
            blind_sensitivity: 1.0,
            encode: EncodeConfig::default(),
        }
    }

    /// Three neighbours each side, finer blocks, wider search.
    pub fn max() -> Self {
        RestoreConfig {
            radius: 3,
            flow: FlowParams {
                block: 6,
                coarse_search: 8,
                refine_search: 3,
                max_levels: 5,
            },
            max_gap: 24,
            ..RestoreConfig::balanced()
        }
    }

    /// Preset by name: `fast`, `balanced` or `max`.
    pub fn preset(name: &str) -> Result<Self, RestoreError> {
        match name {
            "fast" => Ok(Self::fast()),
            "balanced" => Ok(Self::balanced()),
            "max" => Ok(Self::max()),
            o => Err(RestoreError::Config(format!(
                "unknown preset `{o}` (fast|balanced|max)"
            ))),
        }
    }

    /// Rejects settings the engine cannot run with.
    pub fn validate(&self) -> Result<(), RestoreError> {
        let bad = |m: &str| Err(RestoreError::Config(m.into()));
        if self.radius == 0 || self.radius > 8 {
            return bad("radius must be 1..=8");
        }
        if self.flow.block < 4 || self.flow.block > 32 {
            return bad("flow.block must be 4..=32");
        }
        if self.flow.coarse_search < 1 || self.flow.refine_search < 1 {
            return bad("flow search radii must be >= 1");
        }
        if !(0.0..=4.0).contains(&self.denoise) {
            return bad("denoise must be 0..=4");
        }
        if !(0.0..=1.0).contains(&self.broken_threshold) {
            return bad("broken_threshold must be 0..=1");
        }
        if self.encode.crf > 51 {
            return bad("crf must be 0..=51");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_roundtrip_json() {
        for name in ["fast", "balanced", "max"] {
            let c = RestoreConfig::preset(name).unwrap();
            c.validate().unwrap();
            let s = serde_json::to_string(&c).unwrap();
            let back: RestoreConfig = serde_json::from_str(&s).unwrap();
            assert_eq!(back, c);
        }
        let partial: RestoreConfig = serde_json::from_str(r#"{"radius":4}"#).unwrap();
        assert_eq!(partial.radius, 4);
        assert!(RestoreConfig::preset("x").is_err());
    }
}
