# SPDX-License-Identifier: GPL-3.0-or-later
"""OpenVRX research lab: reference decoder, simulator, experiments, golden vectors.

Modules:
    lanes, standards   exact mirrors of the Rust core (openvrx-core)
    recording          .openvrx reader/writer
    dsp                filters and resamplers shared by simulator and decoder
    raster             CVBS raster on the line-locked grid (NTSC/PAL, colour)
    simgen             receive-chain simulator (adapted from C5VRX research/sim.py)
    decoder            reference decoder (I/Q to fields, frames, PNG/MP4)
    colour             simple NTSC/PAL colour decoder
    metrics            SINAD, weighted SNR, PSNR/SSIM, envelope C/N
    diversity          D1..D4 combining
    experiments        S0, S1, S1b, S2, S3
    golden             golden vectors for the Rust crates
"""
__version__ = "0.1.0"
