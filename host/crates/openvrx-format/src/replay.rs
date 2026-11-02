// SPDX-License-Identifier: GPL-3.0-or-later
//! Replaying a recording as a stream of [`RawBlock`]s, interleaved across
//! receivers in stream-time order, paced to real time (or any speed) or as
//! fast as possible. The live pipeline consumes this exactly like USB
//! capture output.

use crate::reader::{RawReader, Recording};
use crate::{FormatError, Result};
use openvrx_core::{clock, RawBlock, RxId};
use std::time::{Duration, Instant};

/// Replay settings.
#[derive(Clone, Debug)]
pub struct ReplayOptions {
    /// Playback speed (1.0 = real time). None: as fast as possible.
    pub speed: Option<f64>,
    /// Bytes per block (rounded down to whole words), like a USB transfer.
    pub block_bytes: usize,
    /// Start time in seconds of stream time.
    pub start_s: f64,
    /// End time in seconds (None: end of the recording).
    pub end_s: Option<f64>,
    /// Start again from `start_s` at the end (demo mode). Stream byte
    /// positions keep increasing across loops.
    pub looping: bool,
    /// Receivers to replay (None: all).
    pub receivers: Option<Vec<RxId>>,
}

impl Default for ReplayOptions {
    fn default() -> Self {
        ReplayOptions {
            speed: Some(1.0),
            block_bytes: 16384,
            start_s: 0.0,
            end_s: None,
            looping: false,
            receivers: None,
        }
    }
}

impl ReplayOptions {
    /// As fast as possible.
    pub fn unpaced() -> Self {
        ReplayOptions {
            speed: None,
            ..Default::default()
        }
    }
}

struct Lane {
    reader: RawReader,
    start_byte: u64,
    end_byte: u64,
    seq: u64,
}

/// Iterator of [`RawBlock`]s; see [`ReplayOptions`].
///
/// `first_byte` is the file offset (plus whole spans while looping) and
/// `host_ns` is the host time when the block was handed out, as for live
/// capture.
pub struct Replay {
    lanes: Vec<Lane>,
    opts: ReplayOptions,
    wall_start: Option<Instant>,
    loops: u64,
    loop_span_s: f64,
}

impl Replay {
    pub(crate) fn new(rec: &Recording, opts: ReplayOptions) -> Result<Replay> {
        if let Some(s) = opts.speed {
            if !(s > 0.0 && s.is_finite()) {
                return Err(FormatError::Invalid(format!("replay speed {s}")));
            }
        }
        if opts.block_bytes == 0 {
            return Err(FormatError::Invalid("block_bytes must be > 0".into()));
        }
        let rxs: Vec<RxId> = match &opts.receivers {
            Some(v) => v.clone(),
            None => rec.receivers().iter().map(|r| r.rx).collect(),
        };
        let mut lanes = Vec::new();
        let mut span = 0.0f64;
        for rx in rxs {
            let mut reader = rec.raw_reader(rx)?;
            let start_byte = reader.seek_time(opts.start_s)?;
            let end_byte = match opts.end_s {
                Some(e) => reader.timeline().byte_of_time(e).min(reader.len_bytes()),
                None => reader.len_bytes(),
            }
            .max(start_byte);
            let tl = reader.timeline();
            span = span.max(tl.time_of_byte(end_byte) - tl.time_of_byte(start_byte));
            lanes.push(Lane {
                reader,
                start_byte,
                end_byte,
                seq: 0,
            });
        }
        Ok(Replay {
            lanes,
            opts,
            wall_start: None,
            loops: 0,
            loop_span_s: span,
        })
    }

    /// Stream time relative to `start_s`, including completed loops.
    fn rel_time(&self, lane: &Lane, byte: u64) -> f64 {
        let tl = lane.reader.timeline();
        tl.time_of_byte(byte) - tl.time_of_byte(lane.start_byte) + self.loops as f64 * self.loop_span_s
    }

    /// Replay time reached so far (seconds after `start_s`).
    pub fn position_s(&self) -> f64 {
        self.lanes
            .iter()
            .map(|l| self.rel_time(l, l.reader.position_bytes()))
            .fold(0.0, f64::max)
    }

    fn next_block(&mut self) -> Result<Option<RawBlock>> {
        loop {
            let pick = self
                .lanes
                .iter()
                .enumerate()
                .filter(|(_, l)| l.reader.position_bytes() < l.end_byte)
                .map(|(i, l)| (i, self.rel_time(l, l.reader.position_bytes())))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(i, _)| i);
            let Some(i) = pick else {
                if self.opts.looping && self.loop_span_s > 0.0 {
                    self.loops += 1;
                    for l in &mut self.lanes {
                        l.reader.seek_byte(l.start_byte)?;
                    }
                    continue;
                }
                return Ok(None);
            };
            let lane = &mut self.lanes[i];
            let room = (lane.end_byte - lane.reader.position_bytes()) as usize;
            let Some(mut block) = lane.reader.read_block(self.opts.block_bytes.min(room))? else {
                return Ok(None);
            };
            let span_bytes = lane.end_byte - lane.start_byte;
            block.first_byte += self.loops * span_bytes;
            block.seq = lane.seq;
            lane.seq += 1;
            let end_byte = lane.reader.position_bytes();
            let t_end = self.rel_time(&self.lanes[i], end_byte);
            let wall_start = *self.wall_start.get_or_insert_with(Instant::now);
            if let Some(speed) = self.opts.speed {
                let target = wall_start + Duration::from_secs_f64(t_end / speed);
                let now = Instant::now();
                if target > now {
                    std::thread::sleep(target - now);
                }
            }
            block.host_ns = clock::now_ns();
            return Ok(Some(block));
        }
    }
}

impl Iterator for Replay {
    type Item = Result<RawBlock>;
    fn next(&mut self) -> Option<Self::Item> {
        self.next_block().transpose()
    }
}
