// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/arc_phy.c at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d. Changes for OpenVRX: the reading
// of phy_param moved to rf.c (so this file builds on the host); AGC memory
// and DC word decoding removed; arc_gain_stage_starts added for the
// console `gains` reply.
//
// Exact library contract (verified by C5VRX by disassembly, 2026-10-06):
// esp-phy-lib 59c1234e929212aec0fdda75769b759951235536, ESP32-C5,
// libphy.a SHA256 dbf33c41...104fffb. phy_gen_rx_gain_table() packs
// RF[20:12], BB[10:4], fine[2:0] with counter c = start[stage] + k,
// BB = BBCODE[c / 6], fine = 5 - c % 6, and runs the last stage up to
// counter 41.
#include "arc_gain.h"

#include <stddef.h>
#include <string.h>

#define LAST_COUNTER 41u

static const uint8_t s_bb_codes[7] = {1u, 3u, 7u, 15u, 31u, 63u, 127u};

static const uint8_t s_spans_24[ARC_RX_STAGE_COUNT] = {15u, 13u, 5u, 8u, 6u, 4u, 4u, 6u, 0u};
static const uint8_t s_start_24[ARC_RX_STAGE_COUNT] = {0u, 11u, 20u, 20u, 22u, 22u, 23u, 22u, 22u};
static const uint16_t s_rf_24[ARC_RX_STAGE_COUNT] = {64u, 100u, 93u, 94u, 107u, 119u, 124u, 125u, 127u};

static const uint8_t s_spans_5g[ARC_RX_STAGE_COUNT] = {10u, 7u, 8u, 5u, 8u, 5u, 4u, 7u, 0u};
static const uint8_t s_start_5g[ARC_RX_STAGE_COUNT] = {12u, 13u, 15u, 12u, 12u, 12u, 12u, 12u, 12u};
static const uint16_t s_rf_5g[ARC_RX_STAGE_COUNT] = {24u, 20u, 0u, 65u, 97u, 162u, 354u, 419u, 487u};

// Entries the generator emits for these spans and start counters.
static unsigned table_entries(const arc_gain_table_t *t)
{
    unsigned n = 0u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i) n += t->spans[i];
    const unsigned s = t->start[ARC_RX_STAGE_COUNT - 1u];
    return n + (LAST_COUNTER >= s ? LAST_COUNTER - s + 1u : 0u);
}

static void finish_table(arc_gain_table_t *table, uint8_t max_index)
{
    const unsigned entries = table_entries(table);
    unsigned top = entries ? entries - 1u : 0u;
    if (top > ARC_VENDOR_GAIN_MAX) top = ARC_VENDOR_GAIN_MAX;
    table->max_index = max_index >= 2u && max_index <= top ? max_index : (uint8_t)top;
    unsigned covered = 0u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i) covered += table->spans[i];
    table->spans[ARC_RX_STAGE_COUNT - 1u] =
        covered <= table->max_index ? (uint8_t)(table->max_index + 1u - covered) : 0u;
}

void arc_gain_table_from_bytes(arc_gain_table_t *table, const uint8_t spans[ARC_RX_STAGE_COUNT],
                               uint8_t max_index)
{
    if (!table) return;
    memset(table, 0, sizeof(*table));
    if (!spans) {
        memcpy(table->spans, s_spans_5g, sizeof(table->spans));
        memcpy(table->start, s_start_5g, sizeof(table->start));
        memcpy(table->rf_code, s_rf_5g, sizeof(table->rf_code));
        table->band5 = true;
        table->runtime_spans_valid = true;   // constants of the generator
        finish_table(table, max_index);
        return;
    }
    bool valid = true;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i)
        if (spans[i] == 0u) { valid = false; break; }
    memcpy(table->spans, valid ? spans : s_spans_24, sizeof(table->spans));
    memcpy(table->start, s_start_24, sizeof(table->start));
    memcpy(table->rf_code, s_rf_24, sizeof(table->rf_code));
    table->runtime_spans_valid = valid;
    finish_table(table, max_index);
}

bool arc_gain_tuple_decode(const arc_gain_table_t *table, uint8_t gain_index, arc_gain_tuple_t *tuple)
{
    if (!table || !tuple || gain_index > table->max_index) return false;
    unsigned start = 0u;
    unsigned stage = ARC_RX_STAGE_COUNT - 1u;
    for (unsigned i = 0; i + 1u < ARC_RX_STAGE_COUNT; ++i) {
        const unsigned end = start + table->spans[i];
        if (gain_index < end) { stage = i; break; }
        start = end;
    }
    const unsigned counter = table->start[stage] + ((unsigned)gain_index - start);
    unsigned coarse = counter / 6u;
    if (coarse > 6u) coarse = 6u;
    tuple->gain_index = gain_index;
    tuple->rf_stage = (uint8_t)stage;
    tuple->rf_code = table->rf_code[stage];
    tuple->bb_code = s_bb_codes[coarse];
    tuple->fine_code = (uint8_t)(5u - (counter % 6u));
    tuple->packed_state = ((uint32_t)tuple->rf_code << 12) | ((uint32_t)tuple->bb_code << 4) | tuple->fine_code;
    return true;
}

unsigned arc_gain_stage_starts(const arc_gain_table_t *table, uint8_t stage_start[ARC_RX_STAGE_COUNT])
{
    unsigned start = 0u, n = 0u;
    for (unsigned i = 0; i < ARC_RX_STAGE_COUNT; ++i) {
        if (start > table->max_index) break;
        stage_start[n++] = (uint8_t)start;
        start += table->spans[i];
    }
    return n;
}
