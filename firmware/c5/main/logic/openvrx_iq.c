// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: 10-bit I/Q captures from the ADC dump engine (pure logic, C5-20).
#include "openvrx_iq.h"

#include <string.h>

static int16_t sign_extend_10(uint32_t v)
{
    v &= 0x3ffu;
    return (int16_t)((v & 0x200u) ? (int)v - 0x400 : (int)v);
}

openvrx_iq_sample_t openvrx_iq_decode(uint32_t word)
{
    openvrx_iq_sample_t s;
    s.i = sign_extend_10(word >> 10);
    s.q = sign_extend_10(word);
    s.gain = (uint8_t)((word >> OPENVRX_IQ_GAIN_SHIFT) & OPENVRX_IQ_GAIN_MASK);
    return s;
}

uint32_t openvrx_iq_sentinel(unsigned index)
{
    // An odd multiplier spreads neighbouring indexes; bit 31 keeps it out of the sample words' range.
    return 0x80000000u | ((0xa5c30000u ^ ((uint32_t)index * 0x9e3779b9u)) & 0x7fffffffu);
}

unsigned openvrx_iq_written(const uint32_t *w, unsigned n)
{
    unsigned k = 0;
    for (unsigned i = 0; i < n; ++i) k += w[i] != openvrx_iq_sentinel(i);
    return k;
}

// Lane code of a 10-bit value on a layout carrying the top `bits`: floor(v / 2^(10 - bits)).
static int lane_code(int v, unsigned shift) { return v >= 0 ? v >> shift : -(((-v) - 1) >> shift) - 1; }

bool openvrx_iq_add(openvrx_agc_acc_t *acc, const uint32_t *w, unsigned n, unsigned bits, int gain, bool gain_strict,
                    openvrx_iq_check_t *chk)
{
    memset(chk, 0, sizeof(*chk));
    if (!n || bits < 2 || bits > 10) return false;
    const unsigned shift = 10u - bits;
    if (acc->values && acc->frac != shift) return false;
    const int hi = (1 << (bits - 1)) - 1, lo = -(1 << (bits - 1));
    openvrx_agc_acc_t add;
    openvrx_agc_acc_reset(&add);
    add.frac = (uint8_t)shift;
    for (unsigned k = 0; k < n; ++k) {
        if (w[k] >> 27) ++chk->high_bits;
        if (k && w[k] == w[k - 1]) ++chk->repeats;
        const openvrx_iq_sample_t s = openvrx_iq_decode(w[k]);
        if (gain >= 0 && s.gain != (unsigned)gain) ++chk->gain_bad;
        const int v[2] = {s.i, s.q};
        for (unsigned a = 0; a < 2; ++a) {
            const int c = lane_code(v[a], shift);
            add.clips += c == hi || c == lo;
            add.rails += v[a] == OPENVRX_IQ_RAIL_HI || v[a] == OPENVRX_IQ_RAIL_LO;
            add.sumsq4 += (uint64_t)((2 * v[a] + 1) * (2 * v[a] + 1));
            add.sum2[a] += 2 * v[a] + 1;
            add.neg[a] += v[a] < 0;
        }
        add.values += 2;
    }
    chk->samples = n;
    chk->rails = add.rails;
    if ((gain_strict && chk->gain_bad) || (n > 1 && chk->repeats == n - 1)) return false;
    acc->frac = add.frac;
    acc->values += add.values;
    acc->clips += add.clips;
    acc->rails += add.rails;
    acc->sumsq4 += add.sumsq4;
    for (unsigned a = 0; a < 2; ++a) {
        acc->sum2[a] += add.sum2[a];
        acc->neg[a] += add.neg[a];
    }
    return true;
}
