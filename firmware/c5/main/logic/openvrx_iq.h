// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: 10-bit I/Q captures from the ADC dump engine (pure logic, C5-20).
//
// The modem's dump engine, the same one rf.c runs in continuous mode to feed
// the lane pads, can also write a short one-shot capture into SRAM: one
// 32-bit word per sample, Q in bits 0..9 and I in bits 10..19 (signed 10-bit,
// the decode of the vendor's own print_dump_data and accumiq), and the gain
// index in force in bits 20..26. The gain loop and the DC servo can read
// those instead of the lane pads: all 10 bits, so a level under one lane step
// is still seen (C5-8), the lanes' clipped share is exact (every sample is
// decoded to the code the lanes carry), the ADC rails are counted on their
// own, and the gain field shows whether the forced index held.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "openvrx_agc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPENVRX_IQ_GAIN_SHIFT 20u
#define OPENVRX_IQ_GAIN_MASK 0x7fu
#define OPENVRX_IQ_RAIL_HI 511
#define OPENVRX_IQ_RAIL_LO (-512)
// Largest capture the firmware takes (words; the engine's RAM holds 16384).
#define OPENVRX_IQ_MAX 1024u

typedef struct {
    int16_t i, q;
    uint8_t gain;   // the gain field (bits 20..26)
} openvrx_iq_sample_t;

openvrx_iq_sample_t openvrx_iq_decode(uint32_t word);

// Fill pattern written before a capture: a word the engine did not write
// still holds it. Bit 31 is always set; what the engine puts in bits 27..31
// is not known (only 0..26 are decoded), so they are counted (high_bits),
// not trusted.
uint32_t openvrx_iq_sentinel(unsigned index);
// Words of w[0..n) that no longer hold their fill pattern.
unsigned openvrx_iq_written(const uint32_t *w, unsigned n);

typedef struct {
    unsigned samples;    // decoded
    unsigned gain_bad;   // samples whose gain field differs from the expected index (0 when not checked)
    unsigned rails;      // component values at an ADC rail (-512 or +511)
    unsigned repeats;    // samples equal to the one before (a stopped engine repeats one word)
    unsigned high_bits;  // samples with any of bits 27..31 set (not a sample word)
} openvrx_iq_check_t;

// Adds the n capture words w to acc as a layout with `bits` per component
// (2 to 10) sees them: the clipped share at the lanes' end codes, the means
// and squares at 10 - bits fraction bits (openvrx_agc_acc_t.frac), and the
// rails. The acc must be empty or hold captures at the same bits. gain >= 0
// counts the words whose gain field differs from it (chk->gain_bad). *chk
// gets the checks above. Returns false, acc unchanged, when the capture
// cannot stand for the receiver: no words, every word the same (the engine
// did not run), or with gain_strict a gain field other than `gain` (the
// index moved during the capture, or the force did not hold).
bool openvrx_iq_add(openvrx_agc_acc_t *acc, const uint32_t *w, unsigned n, unsigned bits, int gain, bool gain_strict,
                    openvrx_iq_check_t *chk);

#ifdef __cplusplus
}
#endif
