// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/arc_phy.h at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d. Changes for OpenVRX: kept only
// the pure gain-table reconstruction (no AGC memory or DC words), renamed
// the header, added the filter byte helper from C5VRX main/dsp.h
// (dsp_filter_code) so both build on the host for unit tests.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARC_RX_STAGE_COUNT 9u
#define ARC_VENDOR_GAIN_MAX 89u
#define ARC_5G_GAIN_MAX 83u     // 84 entries, counters 12..41 in the last stage

// The installed RX gain table, reconstructed exactly as the pinned vendor
// generator builds it (phy_gen_rx_gain_table, esp-phy-lib 59c1234): entry
// k of stage s has counter c = start[s] + k, BB = {1,3,...,127}[c / 6] and
// fine = 5 - c % 6. The 5 GHz band uses its own constant spans, start
// counters and RF codes, with its maximum at phy_param + 0x126.
typedef struct {
    uint8_t spans[ARC_RX_STAGE_COUNT];
    uint8_t start[ARC_RX_STAGE_COUNT];
    uint16_t rf_code[ARC_RX_STAGE_COUNT];
    uint8_t max_index;
    bool band5;
    bool runtime_spans_valid;
} arc_gain_table_t;

typedef struct {
    uint8_t gain_index;
    uint8_t rf_stage;
    uint16_t rf_code;
    uint8_t bb_code;
    uint8_t fine_code;
    uint32_t packed_state;
} arc_gain_tuple_t;

// spans == NULL: the 5 GHz table; otherwise the 2.4 GHz table with those
// runtime spans.
void arc_gain_table_from_bytes(arc_gain_table_t *table, const uint8_t spans[ARC_RX_STAGE_COUNT],
                               uint8_t max_index);
bool arc_gain_tuple_decode(const arc_gain_table_t *table, uint8_t gain_index, arc_gain_tuple_t *tuple);
// First gain index of each RF stage (stage_start[0] = 0). Returns the
// number of stages that start at or below max_index.
unsigned arc_gain_stage_starts(const arc_gain_table_t *table, uint8_t stage_start[ARC_RX_STAGE_COUNT]);

// Second-stage filter byte: calibrated byte with its low six bits moved by
// offset, clamped to 0..60 (C5VRX main/dsp.h dsp_filter_code).
static inline uint8_t arc_filter_code(uint8_t calibrated, int offset)
{
    int code = (calibrated & 63) + offset;
    if (code < 0) code = 0;
    if (code > 60) code = 60;
    return (uint8_t)((calibrated & ~63u) | (unsigned)code);
}

#ifdef __cplusplus
}
#endif
