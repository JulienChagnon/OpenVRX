# SPDX-License-Identifier: GPL-3.0-or-later
#
# Adapted from C5VRX research/sim.py (commit 6bc9e4bbb5bef5948d42a065da27eae39315ad2d),
# C5VRX by Twotoz and the C5VRX contributors, https://github.com/Twotoz/C5VRX,
# GPL-3.0. Kept from the original: FM at 80 MS/s with blanking at the -1.0 MHz
# porch target and 6.7 MHz sync-tip-to-white per unit deviation, the C5
# analog channel (5th-order Butterworth, 10 MHz), thermal noise at a C/N
# measured in that ~20 MHz channel with the mixing rule
# y = (z sqrt(c) + n) / sqrt(1 + c), an optional narrower analog filter after
# the noise, unfiltered decimation, the 4.2-cell gain policy for folded 4-bit
# lanes. Changed: the raster is built on the OpenVRX line-locked grid with
# colour pictures from images or video (raster.py); decimation by any PARLIO
# divider (N = divider / 3) instead of a fixed 40 MS/s; 10-bit ADC
# quantisation with an I DC offset and encoding into every OpenVRX lane preset
# (including angle-only); LO offset, sample-clock ppm, fading (Rayleigh or
# Rician with Doppler, log-normal shadowing), a block AGC, the sampling-phase
# lottery, the 802.11p bus-at-40-MS/s hypothesis; chunked generation for long
# runs; output as .openvrx recordings with truth. The U85Q demodulator, DAC and
# goggle models of the original are not used here.
"""Receive-chain simulator: CVBS -> FM -> C5 channel -> noise -> lanes.

Chain per receiver::

    raster (grid, IRE)  -> band-limited CVBS at the receiver's 80 MS/s bus times
    -> FM: f = porch + LO offset + deviation * 6.7 MHz * IRE / 140
    -> C5 analog channel (Butterworth 5, 10 MHz, complex) at 80 MS/s
    -> + complex noise through the same channel, C/N per that channel
    -> optional 802.11p filter (Butterworth 5, 6 MHz)
    -> keep bus sample k*N (N = divider / 3), no filtering
    -> fading gain, analog gain (AGC), I/Q DC offset
    -> 10-bit ADC (floor, saturate) -> lane preset words (-> glitch model)

The signal and noise parts are generated separately (both linear up to the
ADC), so one capture serves every C/N, fading draw and lane format.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Optional

import numpy as np
from scipy import signal as sg

from . import dsp
from .lanes import IqEncoding, LaneFormat, SampleFormat, words_to_bytes
from .raster import make_raster
from .recording import ReceiverInfo, RecordingWriter
from .standards import get_standard

DEV_PP = 6.7e6          # sync tip to peak white at deviation 1.0 (C5VRX)
PORCH_HZ = -1.0e6       # blanking at the C5VRX AFC porch target
IRE_SPAN = 140.0
CHUNK = 1 << 20


@dataclass
class TxConfig:
    standard: str = "ntsc"
    picture: str = "scene"
    deviation: float = 1.0
    porch_hz: float = PORCH_HZ
    vtx_ppm: float = 0.0
    colour: bool = True
    start_line: Optional[int] = None
    seed: int = 0


class Transmitter:
    """The VTX: a CVBS raster and its FM frequency at any time."""

    def __init__(self, cfg: TxConfig, duration_s: float, margin_lines: int = 3):
        self.cfg = cfg
        self.std = get_standard(cfg.standard)
        rng = np.random.default_rng(cfg.seed + 1000)
        std = self.std
        self.spl = std.samples_per_line
        self.first_frame_line = int(cfg.start_line or rng.integers(1, std.lines_per_frame + 1))
        self.n_lines = int(np.ceil(duration_s * std.line_hz * (1 + abs(cfg.vtx_ppm) * 1e-6))) + 2 * margin_lines + 2
        raster = make_raster(std, cfg.picture, cfg.seed, cfg.colour)
        self.lines = raster.lines(self.first_frame_line, self.n_lines)
        self.flat = self.lines.ravel()
        # Grid position at t = 0: a few lines in, at a random point of a line.
        self.g_offset = margin_lines * self.spl + rng.uniform(0, self.spl)
        self.grid_rate = std.grid_rate_hz * (1 + cfg.vtx_ppm * 1e-6)
        self.hz_per_ire = cfg.deviation * DEV_PP / IRE_SPAN

    def cvbs(self, t: np.ndarray) -> np.ndarray:
        return dsp.interp(self.flat, self.g_offset + np.asarray(t) * self.grid_rate)

    def line_time(self, m) -> np.ndarray:
        """Time of 0H of truth line m (seconds)."""
        return (np.asarray(m, np.float64) * self.spl - self.g_offset) / self.grid_rate

    def frame_line_of(self, m) -> np.ndarray:
        return (self.first_frame_line - 1 + np.asarray(m)) % self.std.lines_per_frame + 1

    def frequency(self, t: np.ndarray) -> np.ndarray:
        return self.cfg.porch_hz + self.hz_per_ire * self.cvbs(t)


@dataclass
class RxConfig:
    lo_offset_hz: float = 0.0
    rx_ppm: float = 0.0
    bus_delay: float = 0.0       # time offset in bus ticks (0..1)
    sample_phase: int = 0        # bus index of the first kept sample (0..N-1)
    p11: bool = False
    bus40: bool = False          # 802.11p hypothesis: bus value changes at 40 MS/s
    bus40_phase: int = 0
    seed: int = 0


@dataclass
class Capture:
    """Signal and noise parts of one receiver at one divider (pre-ADC)."""
    divider: int
    fs: float                   # nominal sample rate
    s: np.ndarray               # unit-power carrier part (complex64)
    w: np.ndarray               # noise part, unit power per 20 MHz channel
    s_old: Optional[np.ndarray]
    w_old: Optional[np.ndarray]
    t: np.ndarray               # sample times (s), float64
    line_a: float               # truth line m has 0H at rx sample line_a + line_b * m
    line_b: float
    delay_samples: float        # channel group delay at the carrier, rx samples
    rx: RxConfig = field(default_factory=RxConfig)

    def __len__(self):
        return len(self.s)


def chain_delay_bus(p11: bool) -> float:
    """Group delay (bus ticks) of the analog chain around the porch frequency."""
    f = np.array([-1.0e6])
    d = dsp.sos_group_delay(dsp.CHAN_SOS, f)[0]
    if p11:
        d += dsp.sos_group_delay(dsp.P11_SOS, f)[0]
    return float(d)


def generate(tx: Transmitter, rx: RxConfig, dividers, duration_s: float, want_old: bool = False) -> dict:
    """Runs the analog chain at 80 MS/s in chunks; returns {divider: Capture}."""
    dividers = [int(d) for d in dividers]
    for d in dividers:
        if d % 3:
            raise ValueError(f"divider {d} is not bus locked (multiple of 3)")
    rng = np.random.default_rng(rx.seed)
    fbus = dsp.BUS_HZ * (1 + rx.rx_ppm * 1e-6)
    nbus = int(duration_s * fbus) + 64
    chan_gain = dsp.sos_power_gain("chan")
    zi_s = np.zeros((dsp.CHAN_SOS.shape[0], 2), complex)
    zi_n = np.zeros_like(zi_s)
    zi_s11 = np.zeros((dsp.P11_SOS.shape[0], 2), complex)
    zi_n11 = np.zeros_like(zi_s11)
    phase = rng.uniform(0, 2 * np.pi)
    power_sum, power_n = 0.0, 0
    parts = {d: {"s": [], "w": [], "so": [], "wo": [], "n": []} for d in dividers}
    tail_s = np.zeros(4, complex)
    tail_w = np.zeros(4, complex)
    for n0 in range(0, nbus, CHUNK):
        n1 = min(nbus, n0 + CHUNK)
        n = np.arange(n0, n1)
        t = (n + rx.bus_delay) / fbus
        f = tx.frequency(t) + rx.lo_offset_hz
        ph = phase + 2 * np.pi * np.cumsum(f) / fbus
        phase = float(ph[-1])
        z, zi_s = sg.sosfilt(dsp.CHAN_SOS, np.exp(1j * ph), zi=zi_s)
        power_sum += float(np.sum(z.real ** 2 + z.imag ** 2))
        power_n += len(z)
        noise = (rng.standard_normal(len(n), dtype=np.float32)
                 + 1j * rng.standard_normal(len(n), dtype=np.float32)) / np.sqrt(2.0)
        w, zi_n = sg.sosfilt(dsp.CHAN_SOS, noise.astype(complex), zi=zi_n)
        w /= np.sqrt(chan_gain)
        if rx.p11:
            z, zi_s11 = sg.sosfilt(dsp.P11_SOS, z, zi=zi_s11)
            w, zi_n11 = sg.sosfilt(dsp.P11_SOS, w, zi=zi_n11)
        # Prepend the previous chunk's tail so index n-1 / n-2 always exists.
        zx = np.concatenate([tail_s, z])
        wx = np.concatenate([tail_w, w])
        tail_s, tail_w = z[-4:].copy(), w[-4:].copy()
        for d in dividers:
            N = d // 3
            k0 = int(np.ceil((n0 - rx.sample_phase) / N))
            k1 = int(np.ceil((n1 - rx.sample_phase) / N))
            idx = np.arange(k0, k1) * N + rx.sample_phase
            idx = idx[idx >= 0]
            if rx.bus40:
                idx_eff = idx - ((idx - rx.bus40_phase) % 2)
                idx_old = idx_eff - 2
            else:
                idx_eff = idx
                idx_old = idx - 1
            loc = idx_eff - n0 + 4
            ok = loc >= 0
            p = parts[d]
            p["s"].append(zx[loc[ok]])
            p["w"].append(wx[loc[ok]])
            p["n"].append(idx_eff[ok])
            if want_old:
                lo = np.maximum(idx_old[ok] - n0 + 4, 0)
                p["so"].append(zx[lo])
                p["wo"].append(wx[lo])
    norm = 1.0 / np.sqrt(power_sum / power_n)
    out = {}
    for d in dividers:
        N = d // 3
        p = parts[d]
        s = (np.concatenate(p["s"]) * norm).astype(np.complex64)
        w = np.concatenate(p["w"]).astype(np.complex64)
        so = (np.concatenate(p["so"]) * norm).astype(np.complex64) if want_old else None
        wo = np.concatenate(p["wo"]).astype(np.complex64) if want_old else None
        nidx = np.concatenate(p["n"]).astype(np.float64)
        t = (nidx + rx.bus_delay) / fbus
        # Truth line m: 0H at time tx.line_time(m); rx sample index k has bus
        # index k*N + sample_phase.
        t0 = tx.line_time(0)
        dt_line = tx.spl / tx.grid_rate
        a = (t0 * fbus - rx.bus_delay - rx.sample_phase) / N
        b = dt_line * fbus / N
        delay = chain_delay_bus(rx.p11) / N
        out[d] = Capture(d, 240e6 / d, s, w, so, wo, t, a, b, delay, rx)
    return out


# ---------------------------------------------------------------------------
# Fading
# ---------------------------------------------------------------------------

@dataclass
class FadingConfig:
    kind: str = "none"          # none | rayleigh | rician
    doppler_hz: float = 150.0
    k_db: float = 6.0           # Rician K
    shadow_db: float = 0.0      # log-normal shadowing sigma
    shadow_tau_s: float = 0.3   # shadowing correlation time
    seed: int = 0


def fading_gain(t: np.ndarray, cfg: FadingConfig) -> np.ndarray:
    """Complex channel gain g(t), E|g|^2 = 1 (before shadowing)."""
    if cfg.kind == "none" and cfg.shadow_db == 0:
        return np.ones(len(t), np.complex64)
    rng = np.random.default_rng(cfg.seed + 77)
    g = np.ones(len(t), complex)
    if cfg.kind in ("rayleigh", "rician"):
        # Sum of sinusoids (Zheng and Xiao), 16 paths.
        M = 16
        theta = rng.uniform(-np.pi, np.pi)
        phi = rng.uniform(-np.pi, np.pi, M)
        psi = rng.uniform(-np.pi, np.pi, M)
        m = np.arange(1, M + 1)
        alpha = (2 * np.pi * m - np.pi + theta) / (4 * M)
        wd = 2 * np.pi * cfg.doppler_hz
        tt = t[:, None]
        ray = (np.cos(wd * tt * np.cos(alpha) + phi).sum(1)
               + 1j * np.cos(wd * tt * np.sin(alpha) + psi).sum(1)) / np.sqrt(M)
        if cfg.kind == "rician":
            k = 10 ** (cfg.k_db / 10)
            th0 = rng.uniform(-np.pi, np.pi)
            los = np.exp(1j * (wd * np.cos(th0) * t + rng.uniform(-np.pi, np.pi)))
            g = np.sqrt(k / (k + 1)) * los + np.sqrt(1 / (k + 1)) * ray
        else:
            g = ray
    if cfg.shadow_db > 0:
        # AR(1) log-normal process at 1 kHz, linearly interpolated.
        dt = 1e-3
        n = int(np.ceil((t[-1] - t[0]) / dt)) + 2
        a = np.exp(-dt / cfg.shadow_tau_s)
        x = np.empty(n)
        x[0] = rng.standard_normal()
        e = rng.standard_normal(n) * np.sqrt(1 - a * a)
        for i in range(1, n):
            x[i] = a * x[i - 1] + e[i]
        tg = t[0] + np.arange(n) * dt
        sh_db = cfg.shadow_db * np.interp(t, tg, x)
        g = g * 10 ** (sh_db / 20)
    return g.astype(np.complex64)


# ---------------------------------------------------------------------------
# Mixing, gain, ADC, lanes
# ---------------------------------------------------------------------------

def mix(cap: Capture, cn_db: float, gain: Optional[np.ndarray] = None, old: bool = False) -> np.ndarray:
    """Analog samples at the ADC input before the analog gain (C5VRX rule)."""
    c = 10 ** (cn_db / 10)
    s = cap.s_old if old else cap.s
    w = cap.w_old if old else cap.w
    sig = s if gain is None else s * gain
    return ((np.sqrt(c) * sig + w) / np.sqrt(1 + c)).astype(np.complex64)


def is_folded(fmt: LaneFormat) -> bool:
    enc = fmt.encoding
    if not isinstance(enc, IqEncoding):
        return False
    bits = enc.i.adc_bits
    return any(a - b != 1 for a, b in zip(bits, bits[1:]))


def agc_target(fmt: LaneFormat, contiguous_fraction: float = 0.45, folded_cells: float = 4.2) -> float:
    """Target RMS radius (carrier plus noise) in ADC units.

    Contiguous top-bit lanes (and angle-only): the whole +-512 ADC window,
    RMS at 0.45 of full scale. Folded lanes ({9,7,6,5}, {9,6,5,4}): C5VRX's
    ultrafine policy, RMS radius of 4.2 cells of the carried LSB (0.525 of the
    folded window), accepting rare folds."""
    if is_folded(fmt):
        enc = fmt.encoding
        return folded_cells * float(1 << enc.i.adc_lsb)
    return contiguous_fraction * 512.0


@dataclass
class AgcConfig:
    mode: str = "static"        # static | tracking
    target: Optional[float] = None  # RMS radius, ADC units (None: agc_target)
    block_s: float = 1e-3
    deadband_db: float = 3.0
    step_db: float = 1.0
    overload_drop_db: float = 6.0


def apply_agc(y: np.ndarray, fs: float, fmt: LaneFormat, cfg: AgcConfig):
    """Returns (gain per sample, [(sample_index, gain_db)] change log)."""
    target = cfg.target if cfg.target is not None else agc_target(fmt)
    if cfg.mode == "static":
        g = target / np.sqrt(np.mean(np.abs(y) ** 2))
        g_db = round(20 * np.log10(g) / cfg.step_db) * cfg.step_db
        return np.full(len(y), 10 ** (g_db / 20), np.float32), [(0, g_db)]
    block = max(16, int(cfg.block_s * fs))
    nb = int(np.ceil(len(y) / block))
    fs_comp = fmt.component_full_scale if fmt.has_magnitude else 512.0
    p0 = np.mean(np.abs(y[: block * 4]) ** 2)
    g_db = round(20 * np.log10(target / np.sqrt(p0)) / cfg.step_db) * cfg.step_db
    gains = np.empty(len(y), np.float32)
    log = [(0, g_db)]
    for b in range(nb):
        seg = y[b * block:(b + 1) * block]
        g = 10 ** (g_db / 20)
        gains[b * block:(b + 1) * block] = g
        p = float(np.mean(seg.real ** 2 + seg.imag ** 2))
        if p <= 0:
            continue
        over = np.mean(np.maximum(np.abs(seg.real), np.abs(seg.imag)) * g > 0.97 * fs_comp)
        want = 20 * np.log10(target / np.sqrt(p))
        new = g_db
        if over > 1e-3:
            new = g_db - cfg.overload_drop_db
        elif abs(want - g_db) > cfg.deadband_db:
            new = round(want / cfg.step_db) * cfg.step_db
        if new != g_db:
            g_db = new
            log.append(((b + 1) * block, g_db))
    return gains, log


def adc10(y_scaled: np.ndarray, dc_i: float = 0.0, dc_q: float = 0.0):
    """10-bit two's complement ADC: floor and saturate."""
    i = np.clip(np.floor(y_scaled.real + dc_i), -512, 511).astype(np.int16)
    q = np.clip(np.floor(y_scaled.imag + dc_q), -512, 511).astype(np.int16)
    return i, q


@dataclass
class GlitchConfig:
    """Sampling-phase lottery: lanes that latch the previous bus value.

    ``old_lanes``: word bits that always read the previous bus sample.
    ``random_lanes``: word bits that read it at random (probability ``p``)."""
    old_lanes: tuple = ()
    random_lanes: tuple = ()
    p: float = 0.5
    seed: int = 0


def encode_words(i, q, fmt: LaneFormat, i_old=None, q_old=None, glitch: Optional[GlitchConfig] = None):
    words = fmt.encode(i, q)
    if glitch is not None and (glitch.old_lanes or glitch.random_lanes):
        old = fmt.encode(i_old, q_old)
        mask_fixed = 0
        for b in glitch.old_lanes:
            mask_fixed |= 1 << b
        words = (words & ~mask_fixed) | (old & mask_fixed)
        if glitch.random_lanes:
            rng = np.random.default_rng(glitch.seed + 5)
            for b in glitch.random_lanes:
                pick = rng.random(len(words)) < glitch.p
                bit = 1 << b
                words = np.where(pick, (words & ~bit) | (old & bit), words)
    dtype = np.uint8 if fmt.word_bytes == 1 else np.uint16
    return words.astype(dtype)


@dataclass
class ReceiverRun:
    """Everything needed to quantise one receiver capture into words."""
    preset: str
    divider: int
    cn_db: float
    dc_i: float = 20.0
    dc_q: float = 0.0
    agc: AgcConfig = field(default_factory=AgcConfig)
    fading: FadingConfig = field(default_factory=FadingConfig)
    glitch: Optional[GlitchConfig] = None


def quantise(cap: Capture, run: ReceiverRun):
    """Capture -> (words, gain log, analog y before gain, per-sample gain)."""
    sf = SampleFormat.from_preset(run.preset, run.divider)
    fmt = sf.lanes
    g = fading_gain(cap.t, run.fading) if (run.fading.kind != "none" or run.fading.shadow_db > 0) else None
    y = mix(cap, run.cn_db, g)
    gains, log = apply_agc(y, cap.fs, fmt, run.agc)
    i, q = adc10(y * gains, run.dc_i, run.dc_q)
    io = qo = None
    if run.glitch is not None:
        yo = mix(cap, run.cn_db, g, old=True)
        io, qo = adc10(yo * gains, run.dc_i, run.dc_q)
    words = encode_words(i, q, fmt, io, qo, run.glitch)
    return words, log, y, gains


# ---------------------------------------------------------------------------
# Recordings
# ---------------------------------------------------------------------------

def simulate_recording(base, tx_cfg: TxConfig, runs, rx_cfgs, duration_s: float, notes: str = ""):
    """Writes a .openvrx recording with one receiver per (run, rx_cfg) pair.

    Truth (rx0's sample times, IRE) goes to NAME.truth.f32. The sidecar's
    ``sim`` block holds every setting plus the raster mapping:
    truth line m (frame line ``first_frame_line + m``, modulo the frame) has
    its 0H at rx0 sample ``line_a + line_b * m`` before the channel delay.
    """
    tx = Transmitter(tx_cfg, duration_s)
    infos, words_all, logs, caps = [], [], [], []
    for k, (run, rxc) in enumerate(zip(runs, rx_cfgs)):
        cap = generate(tx, rxc, [run.divider], duration_s, want_old=run.glitch is not None)[run.divider]
        words, log, _, _ = quantise(cap, run)
        sf = SampleFormat.from_preset(run.preset, run.divider)
        infos.append(ReceiverInfo(rx=k, file="", sample_format=sf, channel_mhz=5732,
                                  offset_khz=rxc.lo_offset_hz / 1e3, gain=int(round(log[0][1])), p11=rxc.p11,
                                  board="sim", antenna=f"sim {run.fading.kind}"))
        words_all.append(words)
        logs.append(log)
        caps.append(cap)
    cap0 = caps[0]
    sim = {
        "generator": "openvrx-lab simgen (adapted from C5VRX research/sim.py)",
        "tx": asdict(tx_cfg),
        "duration_s": duration_s,
        "receivers": [{"run": _run_json(r), "rx": asdict(c)} for r, c in zip(runs, rx_cfgs)],
        "raster": {
            "first_frame_line": tx.first_frame_line,
            "lines": tx.n_lines,
            "line_a": [c.line_a for c in caps],
            "line_b": [c.line_b for c in caps],
            "delay_samples": [c.delay_samples for c in caps],
        },
    }
    w = RecordingWriter(base, infos, source="sim", notes=notes, standard_hint=tx.std.key, sim=sim)
    for k, words in enumerate(words_all):
        sf = infos[k].sample_format
        w.write_raw(k, words_to_bytes(words, sf.lanes.word_bytes))
        bytes_per = sf.lanes.word_bytes
        for idx, gdb in logs[k][1:]:
            w.event(k, "gain", byte=int(idx) * bytes_per, index=float(gdb))
    w.write_truth(tx.cvbs(cap0.t).astype(np.float32), rx=0, delay_samples=cap0.delay_samples)
    return w.close()


def _run_json(r: ReceiverRun) -> dict:
    d = asdict(r)
    return d
