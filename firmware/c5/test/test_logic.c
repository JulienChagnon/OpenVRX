// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: host unit tests for the pure logic in main/logic.
//
// Build and run with test/run_tests.sh (or run_tests.ps1). With the
// argument --tables the program prints the wiring tables used in
// docs/HARDWARE.md instead of running the tests.
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arc_gain.h"
#include "openvrx_agc.h"
#include "openvrx_dc.h"
#include "openvrx_boards.h"
#include "openvrx_cmd.h"
#include "openvrx_lanes.h"
#include "openvrx_sclk_logic.h"
#include "openvrx_selftest.h"
#include "openvrx_settings_logic.h"

static unsigned s_checks, s_failures;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        ++s_checks;                                                                 \
        if (!(cond)) {                                                              \
            ++s_failures;                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                           \
    } while (0)

#define CHECK_STR(a, b) CHECK(strcmp((a), (b)) == 0)

// ---------------------------------------------------------------- lanes

// Mirror of ComponentLanes::encode in lanes.rs, written independently from
// openvrx_preset_lanes: the expected FX2 word for 10-bit I/Q values.
static uint32_t encode_ref(openvrx_preset_t p, int i_adc, int q_adc)
{
    static const uint8_t bits4[3][4] = {{9, 8, 7, 6}, {9, 7, 6, 5}, {9, 6, 5, 4}};
    uint8_t adc[8];
    unsigned n, q_msb, i_msb;
    if (openvrx_preset_word_bytes(p) == 2) {
        n = 8u - (unsigned)p;   // 8x8=0 .. 5x5=3
        for (unsigned k = 0; k < n; ++k) adc[k] = (uint8_t)(9 - k);
        q_msb = 7;
        i_msb = 15;
    } else {
        n = 4;
        memcpy(adc, bits4[p - OPENVRX_PRESET_4X4TOP], 4);
        q_msb = 3;
        i_msb = 7;
    }
    const uint32_t iv = (uint32_t)i_adc & 0x3ffu, qv = (uint32_t)q_adc & 0x3ffu;
    uint32_t w = 0;
    for (unsigned k = 0; k < n; ++k) {
        w |= ((qv >> adc[k]) & 1u) << (q_msb - k);
        w |= ((iv >> adc[k]) & 1u) << (i_msb - k);
    }
    return w;
}

// What the FX2 latches when MODEM_DIAG carries (i, q) and the lanes route
// bus bits to word bits.
static uint32_t word_from_lanes(openvrx_preset_t p, int i_adc, int q_adc)
{
    openvrx_lane_t l[OPENVRX_MAX_LANES];
    const unsigned n = openvrx_preset_lanes(p, l);
    const uint32_t bus = ((uint32_t)q_adc & 0x3ffu) | (((uint32_t)i_adc & 0x3ffu) << 10);
    uint32_t w = 0;
    for (unsigned k = 0; k < n; ++k) w |= ((bus >> l[k].bus_bit) & 1u) << l[k].word_bit;
    return w;
}

static void test_lanes(void)
{
    static const unsigned pins[OPENVRX_PRESET_COUNT] = {16, 14, 12, 10, 8, 8, 8, 8};
    static const unsigned bytes[OPENVRX_PRESET_COUNT] = {2, 2, 2, 2, 1, 1, 1, 1};
    for (unsigned k = 0; k < OPENVRX_PRESET_COUNT; ++k) {
        const openvrx_preset_t p = (openvrx_preset_t)k;
        openvrx_preset_t back;
        CHECK(openvrx_preset_parse(openvrx_preset_name(p), &back) && back == p);
        CHECK(openvrx_preset_data_pins(p) == pins[k]);
        CHECK(openvrx_preset_word_bytes(p) == bytes[k]);
    }
    openvrx_preset_t p;
    CHECK(openvrx_preset_parse("4X4Ultra", &p) && p == OPENVRX_PRESET_4X4ULTRA);
    CHECK(!openvrx_preset_parse("9x9", &p));
    CHECK(!openvrx_preset_parse("8x", &p));
    CHECK(!openvrx_preset_is_iq(OPENVRX_PRESET_ANGLE8));
    openvrx_lane_t l[OPENVRX_MAX_LANES];
    CHECK(openvrx_preset_lanes(OPENVRX_PRESET_ANGLE8, l) == 0);

    // 7x7 lane order: Q bits 3..9 on word bits 1..7, then I bits 3..9 on 9..15.
    CHECK(openvrx_preset_lanes(OPENVRX_PRESET_7X7, l) == 14);
    for (unsigned k = 0; k < 7; ++k) {
        CHECK(l[k].comp == 'Q' && l[k].word_bit == 1 + k && l[k].adc_bit == 3 + k && l[k].bus_bit == 3 + k);
        CHECK(l[7 + k].comp == 'I' && l[7 + k].word_bit == 9 + k && l[7 + k].adc_bit == 3 + k &&
              l[7 + k].bus_bit == 13 + k);
    }
    CHECK(openvrx_preset_word_mask(OPENVRX_PRESET_7X7) == 0xfefeu);
    CHECK(openvrx_preset_word_mask(OPENVRX_PRESET_8X8) == 0xffffu);
    CHECK(openvrx_preset_word_mask(OPENVRX_PRESET_5X5) == 0xf8f8u);
    CHECK(openvrx_preset_word_mask(OPENVRX_PRESET_4X4FINE) == 0xffu);

    // Ultrafine: Q on DIAG[4,5,6,9], I on DIAG[14,15,16,19] (C5VRX rf.c).
    CHECK(openvrx_preset_lanes(OPENVRX_PRESET_4X4ULTRA, l) == 8);
    static const uint8_t ultra_bus[8] = {4, 5, 6, 9, 14, 15, 16, 19};
    for (unsigned k = 0; k < 8; ++k) CHECK(l[k].bus_bit == ultra_bus[k] && l[k].word_bit == k);

    // Every preset against the independent reference encoder, all 10-bit
    // corner values and a sweep.
    for (unsigned k = 0; k < OPENVRX_PRESET_ANGLE8; ++k)
        for (int i = -512; i < 512; i += 7)
            for (int q = -512; q < 512; q += 11)
                if (word_from_lanes((openvrx_preset_t)k, i, q) != encode_ref((openvrx_preset_t)k, i, q)) {
                    CHECK(0);
                    return;
                }
    // lanes.rs test: 7x7 encode(8, -8) leaves the GND lanes 0x0101 clear.
    CHECK((word_from_lanes(OPENVRX_PRESET_7X7, 8, -8) & 0x0101u) == 0);
    // lanes.rs test: 4x4top encode(-512, 0) & 0xf0 == 0x80.
    CHECK((word_from_lanes(OPENVRX_PRESET_4X4TOP, -512, 0) & 0xf0u) == 0x80u);
}

// ---------------------------------------------------------------- boards

static void test_boards(void)
{
    char why[64];
    const openvrx_board_map_t *x = openvrx_board_builtin(OPENVRX_BOARD_XIAO);
    const openvrx_board_map_t *z = openvrx_board_builtin(OPENVRX_BOARD_ZERO);
    CHECK(x && z && !openvrx_board_builtin(OPENVRX_BOARD_CUSTOM));
    CHECK(openvrx_board_validate(x, why, sizeof(why)));
    CHECK(openvrx_board_validate(z, why, sizeof(why)));
    CHECK(openvrx_board_pin_count(x) == 15);
    CHECK(openvrx_board_pin_count(z) == 17);
    // Board-specific pins that must stay free.
    const uint32_t xm = openvrx_board_gpio_mask(x), zm = openvrx_board_gpio_mask(z);
    CHECK((xm & ((1u << 13) | (1u << 14) | (1u << 27) | (1u << 28) | (1u << 6) | (1u << 26))) == 0);
    CHECK((zm & ((1u << 13) | (1u << 14) | (1u << 26) | (1u << 27))) == 0);
    // Clocks avoid strapping pins (2, 7, 25, 27, 28).
    const uint32_t strap = (1u << 2) | (1u << 7) | (1u << 25) | (1u << 27) | (1u << 28);
    CHECK(((1u << x->clk) & strap) == 0 && x->clk == 10);
    CHECK(((1u << z->clk) & strap) == 0 && z->clk == 0);

    // Fits: XIAO up to 7x7 plus all 4x4; Zero everything but angle8.
    for (unsigned k = 0; k < OPENVRX_PRESET_COUNT; ++k) {
        const openvrx_preset_t p = (openvrx_preset_t)k;
        CHECK(openvrx_board_fits(x, p) == (p != OPENVRX_PRESET_8X8 && p != OPENVRX_PRESET_ANGLE8));
        CHECK(openvrx_board_fits(z, p) == (p != OPENVRX_PRESET_ANGLE8));
    }
    CHECK(openvrx_board_default_preset(x) == OPENVRX_PRESET_7X7);
    CHECK(openvrx_board_default_preset(z) == OPENVRX_PRESET_8X8);

    openvrx_route_t r;
    int missing;
    CHECK(openvrx_route(x, OPENVRX_PRESET_8X8, &r, &missing) == OPENVRX_ROUTE_NO_FIT && missing == 0);
    CHECK(openvrx_route(x, OPENVRX_PRESET_ANGLE8, &r, &missing) == OPENVRX_ROUTE_UNSUPPORTED);

    // XIAO 5x5 uses front pins only (no back pads 2..5), clock on D10.
    CHECK(openvrx_route(x, OPENVRX_PRESET_5X5, &r, NULL) == OPENVRX_ROUTE_OK);
    const uint32_t back = (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5);
    CHECK((r.used_mask & back) == 0 && r.n == 10);
    CHECK(r.idle_mask == back);   // back pads driven low in 16-bit wiring
    CHECK(r.spare_mask == 0);     // the 8-bit table only uses front pins of the 16-bit table
    // XIAO 7x7 uses every pin.
    CHECK(openvrx_route(x, OPENVRX_PRESET_7X7, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK((r.used_mask | (1u << r.clk)) == xm && r.idle_mask == 0);
    // PROTOCOL.md example: XIAO Q MSB on G1 then G0.
    CHECK(r.gpio[6] == 1 && r.gpio[5] == 0);
    // XIAO 4x4: front pins only, D0..D4 on the same FX2 pins as in 16-bit.
    CHECK(openvrx_route(x, OPENVRX_PRESET_4X4ULTRA, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK((r.used_mask & back) == 0 && r.n == 8);
    // Spares: back pads and D8/D9, which only the 16-bit wiring uses.
    CHECK(r.spare_mask == (back | (1u << 8) | (1u << 9)));
    for (unsigned wb = 3; wb < 8; ++wb) CHECK(x->w8[wb] == x->w16[wb]);

    // C5-Zero: the 8-bit table is the PB half of the 16-bit table (no rewiring).
    for (unsigned wb = 0; wb < 8; ++wb) CHECK(z->w8[wb] == z->w16[wb]);
    CHECK(openvrx_route(z, OPENVRX_PRESET_7X7, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK(r.idle_mask == ((1u << 28) | (1u << 11)));   // PB0 and PD0 driven low
    CHECK(openvrx_route(z, OPENVRX_PRESET_4X4TOP, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK(r.idle_mask == 0 && (r.spare_mask & (1u << 10)));
    // GPIO11 (499 ohm on the Zero) carries only the I bit 2 lane.
    CHECK(z->w16[8] == 11);

    // NONE route: nothing used, every lane pin spare.
    CHECK(openvrx_route(z, OPENVRX_PRESET_NONE, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK(r.n == 0 && r.spare_mask == (zm & ~(1u << z->clk)));

    // Validation of custom maps.
    openvrx_board_map_t c = {.name = "custom", .clk = 10};
    memset(c.w16, -1, sizeof(c.w16));
    memset(c.w8, -1, sizeof(c.w8));
    CHECK(!openvrx_board_validate(&c, why, sizeof(why)));   // no lanes
    c.w8[0] = 1;
    CHECK(openvrx_board_validate(&c, why, sizeof(why)));
    c.w8[1] = 1;
    CHECK(!openvrx_board_validate(&c, why, sizeof(why)) && strstr(why, "twice"));
    c.w8[1] = 13;
    CHECK(!openvrx_board_validate(&c, why, sizeof(why)) && strstr(why, "not_usable"));
    c.w8[1] = 10;
    CHECK(!openvrx_board_validate(&c, why, sizeof(why)) && strstr(why, "clock"));
    c.w8[1] = 2;
    c.clk = 18;
    CHECK(!openvrx_board_validate(&c, why, sizeof(why)));
    CHECK(openvrx_gpio_usable(0) && openvrx_gpio_usable(12) && openvrx_gpio_usable(23) && openvrx_gpio_usable(28));
    CHECK(!openvrx_gpio_usable(13) && !openvrx_gpio_usable(15) && !openvrx_gpio_usable(22) && !openvrx_gpio_usable(29));
    openvrx_board_id_t id;
    CHECK(openvrx_board_parse("zero", &id) && id == OPENVRX_BOARD_ZERO);
    CHECK(!openvrx_board_parse("Zero", &id));
    CHECK_STR(openvrx_fx2_pin_name(0), "PB0");
    CHECK_STR(openvrx_fx2_pin_name(15), "PD7");
}

// ---------------------------------------------------------------- commands

static bool parse(const char *text, openvrx_cmd_t *cmd, openvrx_err_t *err)
{
    static char line[256];
    const char *word;
    snprintf(line, sizeof(line), "%s", text);
    return openvrx_cmd_parse(line, cmd, err, &word);
}

static void test_cmd(void)
{
    openvrx_cmd_t c;
    openvrx_err_t e;
    CHECK(parse("ver", &c, &e) && c.kind == OPENVRX_CMD_VER);
    CHECK(parse("  ping  ", &c, &e) && c.kind == OPENVRX_CMD_PING);
    CHECK(!parse("ping 1", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);
    CHECK(!parse("bogus", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);
    CHECK(!parse("", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);
    CHECK(!parse("a b c d e f g h i", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);

    CHECK(parse("tune 5732", &c, &e) && c.a.tune.mhz == 5732 && c.a.tune.off_khz == 0);
    CHECK(parse("tune 5917 -150", &c, &e) && c.a.tune.mhz == 5917 && c.a.tune.off_khz == -150);
    CHECK(!parse("tune 6000", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(!parse("tune 5732 3000", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(!parse("tune 57x2", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);
    CHECK(!parse("tune", &c, &e) && e.code == OPENVRX_ERR_SYNTAX);

    CHECK(parse("gain 64", &c, &e) && c.a.gain == 64);
    CHECK(!parse("gain -1", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(parse("gains", &c, &e) && c.kind == OPENVRX_CMD_GAINS);

    CHECK(parse("clk 18", &c, &e) && c.a.clk.action == OPENVRX_CLK_DIV && c.a.clk.div == 18);
    CHECK(parse("clk off", &c, &e) && c.a.clk.action == OPENVRX_CLK_OFF);
    CHECK(parse("clk on", &c, &e) && c.a.clk.action == OPENVRX_CLK_ON);
    CHECK(!parse("clk 5", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(!parse("clk 1024", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(parse("clk 1023", &c, &e) && c.a.clk.div == 1023);

    CHECK(parse("slip", &c, &e) && c.a.slip_n == 1);
    CHECK(parse("slip 3", &c, &e) && c.a.slip_n == 3);
    CHECK(!parse("slip 0", &c, &e) && e.code == OPENVRX_ERR_RANGE);

    CHECK(parse("p11 on", &c, &e) && c.a.p11_on);
    CHECK(parse("p11 0", &c, &e) && !c.a.p11_on);
    CHECK(!parse("p11 maybe", &c, &e));

    CHECK(parse("filter cal", &c, &e) && c.a.filter.code == OPENVRX_FILTER_CAL && c.a.filter.skirt == 0);
    CHECK(parse("filter 16 32", &c, &e) && c.a.filter.code == 16 && c.a.filter.skirt == 32);
    CHECK(parse("filter cal 8", &c, &e) && c.a.filter.code == OPENVRX_FILTER_CAL && c.a.filter.skirt == 8);
    CHECK(!parse("filter 64", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(!parse("filter 10 61", &c, &e) && e.code == OPENVRX_ERR_RANGE);

    CHECK(parse("bw 20", &c, &e) && c.a.bw_mhz == 20);
    CHECK(!parse("bw 80", &c, &e) && e.code == OPENVRX_ERR_RANGE);

    CHECK(parse("drive 1", &c, &e) && c.a.drive.lanes == 1 && c.a.drive.clk == -1);
    CHECK(parse("drive 0 3", &c, &e) && c.a.drive.lanes == 0 && c.a.drive.clk == 3);
    CHECK(!parse("drive 4", &c, &e) && e.code == OPENVRX_ERR_RANGE);

    CHECK(parse("selftest walk", &c, &e) && c.a.selftest.mode == OPENVRX_ST_WALK && c.a.selftest.phase_us == 1000);
    CHECK(parse("selftest count 50", &c, &e) && c.a.selftest.mode == OPENVRX_ST_COUNT && c.a.selftest.phase_us == 50);
    CHECK(parse("selftest const 3fff", &c, &e) && c.a.selftest.mode == OPENVRX_ST_CONST && c.a.selftest.value == 0x3fff);
    CHECK(parse("selftest const 0x00A5", &c, &e) && c.a.selftest.value == 0xa5);
    CHECK(parse("selftest off", &c, &e) && c.a.selftest.mode == OPENVRX_ST_OFF);
    CHECK(!parse("selftest off 5", &c, &e));
    CHECK(!parse("selftest const", &c, &e));
    CHECK(!parse("selftest const xyz", &c, &e));
    CHECK(!parse("selftest walk 5", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(!parse("selftest dance", &c, &e));

    CHECK(parse("board xiao", &c, &e) && c.a.board == OPENVRX_BOARD_XIAO);
    CHECK(!parse("board pico", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(parse("lanes angle8", &c, &e) && c.a.preset == OPENVRX_PRESET_ANGLE8);
    CHECK(!parse("lanes 3x3", &c, &e) && e.code == OPENVRX_ERR_RANGE);
    CHECK(parse("save", &c, &e) && parse("reboot", &c, &e) && parse("dl", &c, &e));

    // ERR formatting echoes the command word.
    char line[64] = "tune 9999";
    const char *word;
    char out[160];
    CHECK(!openvrx_cmd_parse(line, &c, &e, &word));
    openvrx_format_err(out, sizeof(out), word, &e);
    CHECK_STR(out, "ERR tune range MHz out of 5180..5945");

    long v;
    uint32_t h;
    CHECK(openvrx_parse_int("-42", -100, 100, &v) && v == -42);
    CHECK(!openvrx_parse_int("-", -100, 100, &v));
    CHECK(!openvrx_parse_int("4 2", -100, 100, &v));
    CHECK(!openvrx_parse_int("99999999999", 0, 100, &v));
    CHECK(openvrx_parse_hex("FFFFFFFF", &h) && h == 0xffffffffu);
    CHECK(openvrx_parse_hex("000000001", &h) && h == 1);
    CHECK(!openvrx_parse_hex("123456789", &h));
    CHECK(!openvrx_parse_hex("0x", &h));

    // Line assembly: CRLF, overflow, a line after an overflow.
    openvrx_linebuf_t lb;
    openvrx_linebuf_init(&lb);
    const char *in = "ping\r\n";
    openvrx_line_status_t st = OPENVRX_LINE_NONE;
    for (const char *p = in; *p; ++p) st = openvrx_linebuf_feed(&lb, *p);
    CHECK(st == OPENVRX_LINE_READY && strcmp(lb.buf, "ping") == 0);
    for (unsigned k = 0; k < OPENVRX_LINE_MAX + 5; ++k) st = openvrx_linebuf_feed(&lb, 'a');
    CHECK(st == OPENVRX_LINE_NONE);
    CHECK(openvrx_linebuf_feed(&lb, '\n') == OPENVRX_LINE_OVERFLOW);
    for (const char *p = "ver\n"; *p; ++p) st = openvrx_linebuf_feed(&lb, *p);
    CHECK(st == OPENVRX_LINE_READY && strcmp(lb.buf, "ver") == 0);
    // Exactly OPENVRX_LINE_MAX characters still fit.
    for (unsigned k = 0; k < OPENVRX_LINE_MAX; ++k) st = openvrx_linebuf_feed(&lb, 'b');
    CHECK(openvrx_linebuf_feed(&lb, '\n') == OPENVRX_LINE_READY && strlen(lb.buf) == OPENVRX_LINE_MAX);
}

// ---------------------------------------------------------------- selftest

static void test_selftest(void)
{
    const unsigned n = 14;
    CHECK(openvrx_walk_phases(n) == 16);
    CHECK(openvrx_selftest_bits(OPENVRX_ST_WALK, 0, n, 0) == 0);
    CHECK(openvrx_selftest_bits(OPENVRX_ST_WALK, 1, n, 0) == 0x3fffu);
    for (unsigned j = 0; j < n; ++j) CHECK(openvrx_selftest_bits(OPENVRX_ST_WALK, 2 + j, n, 0) == 1u << j);
    CHECK(openvrx_selftest_bits(OPENVRX_ST_WALK, 16, n, 0) == 0);    // wraps
    CHECK(openvrx_selftest_bits(OPENVRX_ST_WALK, 17, n, 0) == 0x3fffu);
    CHECK(openvrx_selftest_bits(OPENVRX_ST_COUNT, 0x12345, n, 0) == (0x12345u & 0x3fffu));
    CHECK(openvrx_selftest_bits(OPENVRX_ST_CONST, 99, n, 0xa5) == 0xa5);
    CHECK(openvrx_selftest_const_fits(0x3fff, 14) && !openvrx_selftest_const_fits(0x4000, 14));
    CHECK(openvrx_selftest_const_fits(0xffff, 16));

    // Lane j lands on the j-th used word bit: for XIAO 7x7 lane 0 is PB1.
    openvrx_route_t r;
    CHECK(openvrx_route(openvrx_board_builtin(OPENVRX_BOARD_XIAO), OPENVRX_PRESET_7X7, &r, NULL) == OPENVRX_ROUTE_OK);
    CHECK(openvrx_lane_bits_to_word(&r, 1u) == 1u << 1);
    CHECK(openvrx_lane_bits_to_word(&r, 1u << 7) == 1u << 9);
    CHECK(openvrx_lane_bits_to_word(&r, 0x3fffu) == 0xfefeu);
    CHECK(openvrx_lane_bits_to_gpio(&r, 1u) == 1u << 3);          // PB1 = MTDI = GPIO3
    CHECK(openvrx_lane_bits_to_gpio(&r, 0x3fffu) == r.used_mask);
}

// ---------------------------------------------------------------- slip

// Divider model: a counter that wraps every `period` PLL ticks and latches
// the register value at each wrap. The output rises at the wrap (rise_half
// false) or half a period later (true). The software side follows
// openvrx_slip_plan with a polling latency.
typedef struct {
    uint64_t t;             // current tick
    uint32_t reg;           // divider register value (period in ticks)
    uint32_t period;        // latched period
    uint64_t period_start;  // tick of the last wrap
    bool rise_half;
    uint64_t wraps[64];
    uint32_t periods[64];
    unsigned n_wraps;
} divider_t;

static void div_step(divider_t *d)
{
    ++d->t;
    if (d->t - d->period_start == d->period) {
        if (d->n_wraps < 64) {
            d->wraps[d->n_wraps] = d->t;
            d->periods[d->n_wraps] = d->period;
            ++d->n_wraps;
        }
        d->period_start = d->t;
        d->period = d->reg;
    }
}

static int div_level(const divider_t *d)
{
    const uint64_t pos = d->t - d->period_start;
    const bool first_half = pos < d->period / 2;
    return d->rise_half ? !first_half : first_half;
}

static void div_wait(divider_t *d, uint64_t ticks)
{
    for (uint64_t k = 0; k < ticks; ++k) div_step(d);
}

// Returns the phase shift (mod 3) of the wraps after the slip relative to
// the wrap grid before it, or -1 when the period sequence is wrong.
static int simulate_slip(uint32_t n, uint32_t start_offset, uint32_t latency, bool rise_half, int jitter)
{
    divider_t d = {.reg = n, .period = n, .rise_half = rise_half};
    div_wait(&d, start_offset);
    const uint64_t grid = d.period_start;   // a wrap of the N grid
    d.n_wraps = 0;
    openvrx_slip_plan_t plan;
    if (!openvrx_slip_plan(n, &plan)) return -1;
    d.reg = plan.long_div;
    div_wait(&d, plan.settle_ticks);
    // Wait for low, then high, as the firmware polls the pad.
    unsigned guard = 0;
    while (div_level(&d) != 0 && guard++ < plan.edge_timeout_ticks) div_step(&d);
    while (div_level(&d) != 1 && guard++ < 2 * plan.edge_timeout_ticks) div_step(&d);
    div_wait(&d, latency);   // the edge is seen late
    const uint64_t r = d.t;
    div_wait(&d, plan.stretch_at_ticks);
    d.reg = plan.long_div + 1;
    div_wait(&d, (uint64_t)((int64_t)(plan.restore_at_ticks - plan.stretch_at_ticks) + jitter));
    d.reg = n;
    (void)r;
    div_wait(&d, 4u * plan.long_div);
    unsigned stretched = 0;
    for (unsigned k = 0; k < d.n_wraps; ++k) {
        const uint32_t p = d.periods[k];
        if (p == plan.long_div + 1) ++stretched;
        else if (p != plan.long_div && p != n) return -1;
    }
    if (stretched != 1 || d.period != n) return -1;
    return (int)((d.period_start - grid) % 3u);
}

static void test_slip(void)
{
    CHECK(openvrx_clock_hz(18) == 13333333u);
    CHECK(openvrx_clock_hz(6) == 40000000u);
    CHECK(openvrx_div_bus_locked(9) && !openvrx_div_bus_locked(10));
    CHECK(openvrx_ticks_to_cycles(1200, 240000000u) == 1200);
    CHECK(openvrx_ticks_to_cycles(1200, 160000000u) == 800);
    CHECK(openvrx_ticks_to_cycles(1, 160000000u) == 1);
    openvrx_slip_plan_t plan;
    CHECK(!openvrx_slip_plan(0, &plan) && !openvrx_slip_plan(1200, &plan));
    CHECK(openvrx_slip_plan(1023, &plan) && plan.long_div % 3 == 0 && plan.long_div > 1023);

    // Every divider class, every start position within a period, polling
    // latencies up to 0.8 us (192 ticks), +-100 ticks of jitter between the
    // two writes and both edge conventions: exactly one stretched period and
    // a shift of +1 tick.
    static const uint32_t divs[] = {6, 9, 12, 18, 30, 1020};
    for (unsigned di = 0; di < sizeof(divs) / sizeof(divs[0]); ++di)
        for (uint32_t off = 0; off < divs[di]; off += divs[di] > 30 ? 37 : 1)
            for (uint32_t lat = 0; lat <= 192; lat += 16)
                for (int jit = -100; jit <= 100; jit += 50)
                    for (int half = 0; half < 2; ++half) {
                        const int shift = simulate_slip(divs[di], divs[di] * 3 + off, lat, half != 0, jit);
                        if (shift != 1) {
                            fprintf(stderr, "slip div=%u off=%u lat=%u jit=%d half=%d shift=%d\n", divs[di], off,
                                    lat, jit, half, shift);
                            CHECK(0);
                            return;
                        }
                    }
    // The model does catch the failure the margins exist for: a latency that
    // puts the first write just before a wrap, plus jitter that puts the
    // second just after the next one, stretches two periods.
    openvrx_slip_plan_t p18;
    CHECK(openvrx_slip_plan(18, &p18));
    bool caught = false;
    for (uint32_t lat = p18.stretch_at_ticks - 4; lat <= p18.stretch_at_ticks + 4; ++lat)
        caught |= simulate_slip(18, 54, lat, true, 6) != 1;
    CHECK(caught);
}

// ---------------------------------------------------------------- settings, gain table

static void test_settings_gain(void)
{
    openvrx_settings_t s;
    openvrx_settings_default(&s, OPENVRX_BOARD_XIAO, OPENVRX_PRESET_7X7);
    CHECK(openvrx_settings_valid(&s));
    openvrx_settings_t t = s;
    t.div = 5;
    CHECK(!openvrx_settings_valid(&t));
    t = s;
    t.preset = OPENVRX_PRESET_NONE;
    CHECK(openvrx_settings_valid(&t));
    t.preset = 42;
    CHECK(!openvrx_settings_valid(&t));
    t = s;
    t.filter_code = -2;
    CHECK(!openvrx_settings_valid(&t));
    t = s;
    t.version = 0;
    CHECK(!openvrx_settings_valid(&t));
    t = s;
    t.agc = 2;
    CHECK(!openvrx_settings_valid(&t));
    // Blobs: the current layout, and a version 1 blob upgraded with the AGC on.
    CHECK(s.agc == 1 && sizeof(s) > OPENVRX_SETTINGS_V1_SIZE);
    openvrx_settings_t u;
    CHECK(openvrx_settings_from_blob(&s, (unsigned)sizeof(s), &u) && u.gain == s.gain);
    uint8_t v1[OPENVRX_SETTINGS_V1_SIZE];
    t = s;
    t.version = 1;
    t.gain = 56;
    memcpy(v1, &t, sizeof(v1));
    memset(&u, 0, sizeof(u));
    CHECK(openvrx_settings_from_blob(v1, (unsigned)sizeof(v1), &u));
    CHECK(u.version == OPENVRX_SETTINGS_VERSION && u.agc == 1 && u.gain == 56 && u.mhz == s.mhz);
    CHECK(u.dc == 1);
    // Version 2: same blob size as version 3 (padding), told apart by the
    // version byte, upgraded with DC on; agc kept as stored.
    CHECK(sizeof(s) == 18u && OPENVRX_SETTINGS_V2_FIELDS == offsetof(openvrx_settings_t, dc));
    uint8_t v2[18];
    t = s;
    t.version = 2;
    t.agc = 0;
    t.gain = 61;
    memcpy(v2, &t, sizeof(v2));
    v2[OPENVRX_SETTINGS_V2_FIELDS] = 0xAA;   // padding garbage
    memset(&u, 0, sizeof(u));
    CHECK(openvrx_settings_from_blob(v2, (unsigned)sizeof(v2), &u));
    CHECK(u.version == OPENVRX_SETTINGS_VERSION && u.dc == 1 && u.agc == 0 && u.gain == 61);
    t = s;
    t.dc = 0;
    CHECK(openvrx_settings_from_blob(&t, (unsigned)sizeof(t), &u) && u.dc == 0);
    t.dc = 2;
    CHECK(!openvrx_settings_valid(&t));
    v1[0] = 3;
    CHECK(!openvrx_settings_from_blob(v1, (unsigned)sizeof(v1), &u));
    CHECK(!openvrx_settings_from_blob(v1, 5, &u));

    arc_gain_table_t g;
    arc_gain_table_from_bytes(&g, NULL, 83);
    CHECK(g.band5 && g.max_index == 83);
    uint8_t starts[ARC_RX_STAGE_COUNT];
    const unsigned ns = arc_gain_stage_starts(&g, starts);
    CHECK(ns == 9 && starts[0] == 0 && starts[8] == 54);   // top RF stage starts at G54 (C5VRX)
    arc_gain_tuple_t tup;
    CHECK(arc_gain_tuple_decode(&g, 83, &tup) && tup.rf_stage == 8 && tup.bb_code == 127 && tup.fine_code == 0);
    CHECK(!arc_gain_tuple_decode(&g, 84, &tup));
    arc_gain_table_from_bytes(&g, NULL, 200);   // out of range: generator maximum
    CHECK(g.max_index == 83);
    CHECK(arc_filter_code(0xc0 | 20, 50) == (0xc0 | 60));
    CHECK(arc_filter_code(0x40 | 5, -10) == 0x40);
}

// ---------------------------------------------------------------- tables for HARDWARE.md

static void print_tables(void)
{
    for (unsigned b = 0; b < 2; ++b) {
        const openvrx_board_map_t *m = openvrx_board_builtin((openvrx_board_id_t)b);
        printf("\n### %s\n", m->name);
        for (unsigned k = 0; k < OPENVRX_PRESET_ANGLE8; ++k) {
            const openvrx_preset_t p = (openvrx_preset_t)k;
            openvrx_route_t r;
            if (openvrx_route(m, p, &r, NULL) != OPENVRX_ROUTE_OK) continue;
            printf("\n#### %s on %s (%u-bit FX2 words)\n\n", openvrx_preset_name(p), m->name,
                   8 * openvrx_preset_word_bytes(p));
            printf("| FX2 pin | Carries | C5 pin |\n|---|---|---|\n");
            const unsigned width = 8 * openvrx_preset_word_bytes(p);
            const int8_t *table = width == 16 ? m->w16 : m->w8;
            for (int wb = (int)width - 1; wb >= 0; --wb) {
                int lane = -1;
                for (unsigned j = 0; j < r.n; ++j) if (r.lane[j].word_bit == wb) lane = (int)j;
                if (lane >= 0)
                    printf("| %s | %c bit %u | GPIO%d |\n", openvrx_fx2_pin_name((unsigned)wb), r.lane[lane].comp,
                           r.lane[lane].adc_bit, r.gpio[lane]);
                else if (table[wb] >= 0)
                    printf("| %s | unused (driven low) | GPIO%d |\n", openvrx_fx2_pin_name((unsigned)wb), table[wb]);
                else
                    printf("| %s | unused | GND |\n", openvrx_fx2_pin_name((unsigned)wb));
            }
            printf("| IFCLK | sample clock | GPIO%d |\n", m->clk);
        }
    }
}

// ---------------------------------------------------------------- agc

// Pad word for 4x4top component values (two's complement nibbles) on a
// plan whose lane k sits on GPIO k.
static uint32_t pads_4x4top(const openvrx_lane_t *lanes, unsigned n, int vi, int vq)
{
    uint32_t w = 0;
    for (unsigned k = 0; k < n; ++k) {
        const int v = lanes[k].comp == 'I' ? vi : vq;
        const unsigned bit = lanes[k].adc_bit - 6u;   // 4x4top carries ADC bits 9..6
        if (((unsigned)v >> bit) & 1u) w |= 1u << k;
    }
    return w;
}

static void test_agc(void)
{
    openvrx_lane_t lanes[OPENVRX_MAX_LANES];
    int8_t gpio[OPENVRX_MAX_LANES];
    const unsigned n = openvrx_preset_lanes(OPENVRX_PRESET_4X4TOP, lanes);
    for (unsigned k = 0; k < n; ++k) gpio[k] = (int8_t)k;
    openvrx_agc_plan_t plan;
    CHECK(openvrx_agc_plan(OPENVRX_PRESET_4X4TOP, lanes, gpio, n, &plan) && plan.bits == 4);
    CHECK(!openvrx_agc_plan(OPENVRX_PRESET_4X4FINE, lanes, gpio, n, &plan));
    CHECK(!openvrx_agc_plan(OPENVRX_PRESET_ANGLE8, lanes, gpio, 0, &plan));
    CHECK(openvrx_agc_plan(OPENVRX_PRESET_4X4TOP, lanes, gpio, n, &plan));

    // Decoding: every value pair round-trips through the pads.
    openvrx_agc_acc_t a;
    for (int vi = -8; vi <= 7; ++vi)
        for (int vq = -8; vq <= 7; ++vq) {
            openvrx_agc_acc_reset(&a);
            openvrx_agc_acc_add(&a, &plan, pads_4x4top(lanes, n, vi, vq));
            const uint64_t want = (uint64_t)((2 * vi + 1) * (2 * vi + 1) + (2 * vq + 1) * (2 * vq + 1));
            CHECK(a.sumsq4 == want);
            CHECK(a.clips == (unsigned)((vi == -8 || vi == 7) + (vq == -8 || vq == 7)));
        }
    // Hard clipped (Nov 2 close-in capture): all corners, overload.
    openvrx_agc_acc_reset(&a);
    for (int k = 0; k < 1000; ++k) openvrx_agc_acc_add(&a, &plan, pads_4x4top(lanes, n, k & 1 ? 7 : -8, k & 2 ? 7 : -8));
    CHECK(openvrx_agc_clip_share(&a) > 0.99f);
    CHECK(openvrx_agc_next(56, 0, 77, 4, openvrx_agc_clip_share(&a), openvrx_agc_rms_steps(&a)) == 56 - OPENVRX_AGC_MAX_STEP);
    // Under one step (range test): only 0 and -1, raise fast.
    openvrx_agc_acc_reset(&a);
    for (int k = 0; k < 1000; ++k) openvrx_agc_acc_add(&a, &plan, pads_4x4top(lanes, n, k & 1 ? 0 : -1, k & 2 ? 0 : -1));
    CHECK(openvrx_agc_rms_steps(&a) < 0.6f && openvrx_agc_clip_share(&a) == 0.0f);
    CHECK(openvrx_agc_next(56, 0, 77, 4, 0.0f, openvrx_agc_rms_steps(&a)) == 56 + OPENVRX_AGC_MAX_STEP);
    // On target (4.4 steps), inside the deadband, small errors: hold or one step.
    CHECK(openvrx_agc_next(50, 0, 77, 4, 0.01f, 4.4f) == 50);
    CHECK(openvrx_agc_next(50, 0, 77, 4, 0.01f, 5.0f) == 50);
    CHECK(openvrx_agc_next(50, 0, 77, 4, 0.01f, 3.4f) == 54);
    CHECK(openvrx_agc_next(50, 0, 77, 4, 0.0f, 3.9f) == 50);
    // Weak but clipping (interference spikes): never raised.
    CHECK(openvrx_agc_next(50, 0, 77, 4, 0.10f, 2.0f) == 50);
    // Limits.
    CHECK(openvrx_agc_next(1, 0, 77, 4, 1.0f, 7.0f) == 0);
    CHECK(openvrx_agc_next(76, 0, 77, 4, 0.0f, 0.5f) == 77);
    // 7x7: full scale 64 steps, target about 35.
    CHECK(openvrx_agc_next(50, 0, 77, 7, 0.0f, 35.0f) == 50);
    CHECK(openvrx_agc_next(50, 0, 77, 7, 0.0f, 8.0f) == 54);

    // Command.
    char line[32] = "agc off";
    openvrx_cmd_t cmd;
    openvrx_err_t err;
    const char *word;
    CHECK(openvrx_cmd_parse(line, &cmd, &err, &word) && cmd.kind == OPENVRX_CMD_AGC && !cmd.a.agc_on);
    strcpy(line, "agc on");
    CHECK(openvrx_cmd_parse(line, &cmd, &err, &word) && cmd.a.agc_on);
    strcpy(line, "agc maybe");
    CHECK(!openvrx_cmd_parse(line, &cmd, &err, &word));
}

// ---------------------------------------------------------------- dc

// Pad word for component values v (two's complement, `bits` wide) on a
// top-bit plan whose lane k sits on GPIO k.
static uint32_t pads_top(const openvrx_lane_t *lanes, unsigned n, unsigned bits, int vi, int vq)
{
    uint32_t w = 0;
    for (unsigned k = 0; k < n; ++k) {
        const int v = lanes[k].comp == 'I' ? vi : vq;
        const unsigned bit = lanes[k].adc_bit - (10u - bits);
        if (((unsigned)v >> bit) & 1u) w |= 1u << k;
    }
    return w;
}

static uint32_t s_rng = 12345u;
static float frand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (float)((s_rng >> 8) + 0.5) / 16777216.0f;
}
static float gauss(void)
{
    const float u = frand(), v = frand();
    return sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * v);
}

// The receiver ahead of the lane cut: DC = dc0 + slope * own trim + cross *
// slope * other trim (ADC units), Gaussian noise, an optional constant
// envelope carrier at a random phase per sample (FM video averages out).
typedef struct {
    float dc0[2], slope[2], cross, noise, carrier;
    unsigned bits;
    openvrx_lane_t lanes[OPENVRX_MAX_LANES];
    unsigned n;
    openvrx_agc_plan_t plan;
} dc_plant_t;

static void plant_init(dc_plant_t *p, openvrx_preset_t preset)
{
    memset(p, 0, sizeof(*p));
    int8_t gpio[OPENVRX_MAX_LANES];
    p->n = openvrx_preset_lanes(preset, p->lanes);
    for (unsigned k = 0; k < p->n; ++k) gpio[k] = (int8_t)k;
    CHECK(openvrx_agc_plan(preset, p->lanes, gpio, p->n, &p->plan));
    p->bits = p->plan.bits;
}

static float plant_dc(const dc_plant_t *p, const int trim[2], unsigned a)
{
    return p->dc0[a] + p->slope[a] * (float)trim[a] + p->cross * p->slope[a] * (float)trim[1 - a];
}

static void plant_measure_n(const dc_plant_t *p, const int trim[2], openvrx_agc_acc_t *acc, unsigned reads)
{
    openvrx_agc_acc_reset(acc);
    const int shift = 10 - (int)p->bits;
    for (unsigned k = 0; k < reads; ++k) {
        const float ph = 6.2831853f * frand();
        int v[2];
        for (unsigned a = 0; a < 2; ++a) {
            const float x = plant_dc(p, trim, a) + p->noise * gauss() + p->carrier * (a ? sinf(ph) : cosf(ph));
            int code = (int)floorf(x);
            code = code < -512 ? -512 : code > 511 ? 511 : code;
            v[a] = code >> shift;   // arithmetic: floor
        }
        openvrx_agc_acc_add(acc, &p->plan, pads_top(p->lanes, p->n, p->bits, v[0], v[1]));
    }
}

static void plant_measure(const dc_plant_t *p, const int trim[2], openvrx_agc_acc_t *acc)
{
    plant_measure_n(p, trim, acc, OPENVRX_AGC_READS);
}

static void meas_from(const dc_plant_t *p, const openvrx_agc_acc_t *acc, openvrx_dc_meas_t *m)
{
    m->step_adc = (float)(1u << (10u - p->bits));
    m->rms_ac_adc = openvrx_agc_rms_ac_steps(acc) * m->step_adc;
    m->reads = acc->values / 2u;
    for (unsigned a = 0; a < 2; ++a) {
        m->mean_adc[a] = openvrx_agc_mean_steps(acc, a) * m->step_adc;
        m->neg_share[a] = openvrx_agc_neg_share(acc, a);
    }
}

// Runs the servo on one row for `updates` measurements; returns the largest
// true |DC| over the last `tail` updates.
static float dc_run(dc_plant_t *p, openvrx_dc_servo_t *s, openvrx_dc_rows_t *rows, unsigned row, uint32_t epoch,
                    unsigned updates, unsigned tail, float wander)
{
    static const int base[2] = {256, 256};
    float worst = 0.0f;
    for (unsigned u = 0; u < updates; ++u) {
        int trim[2];
        openvrx_dc_rows_claim(rows, row, trim);
        openvrx_agc_acc_t acc;
        plant_measure(p, trim, &acc);
        openvrx_dc_meas_t m;
        meas_from(p, &acc, &m);
        int step[2], applied[2];
        bool limited[2];
        openvrx_dc_servo_decide(s, row, epoch, &m, step);
        openvrx_dc_rows_step(rows, row, base, step, applied, limited);
        openvrx_dc_servo_applied(s, applied, limited);
        if (getenv("DC_TRACE"))
            printf("row %u u %2u trim %4d/%4d dc %7.1f/%7.1f mean %7.1f/%7.1f neg %.3f/%.3f step %3d/%3d pol %d/%d\n",
                   row, u, trim[0], trim[1], (double)plant_dc(p, trim, 0), (double)plant_dc(p, trim, 1),
                   (double)m.mean_adc[0], (double)m.mean_adc[1], (double)m.neg_share[0], (double)m.neg_share[1],
                   applied[0], applied[1], s->pol[0], s->pol[1]);
        if (u + tail >= updates) {
            openvrx_dc_rows_claim(rows, row, trim);
            for (unsigned a = 0; a < 2; ++a) {
                const float d = fabsf(plant_dc(p, trim, a));
                if (d > worst) worst = d;
            }
        }
        p->dc0[0] += wander * gauss();
        p->dc0[1] += wander * gauss();
    }
    return worst;
}

static float dc_clip_now(dc_plant_t *p, openvrx_dc_rows_t *rows, unsigned row)
{
    int trim[2];
    openvrx_dc_rows_claim(rows, row, trim);
    openvrx_agc_acc_t acc;
    plant_measure(p, trim, &acc);
    return openvrx_agc_clip_share(&acc);
}

static void test_dc(void)
{
    // DC_SEED=n reruns the simulations with another noise sequence.
    if (getenv("DC_SEED")) s_rng = (uint32_t)strtoul(getenv("DC_SEED"), NULL, 10);
    // Accumulator: mean, sign share and RMS around the mean.
    dc_plant_t p;
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    openvrx_agc_acc_t acc;
    openvrx_agc_acc_reset(&acc);
    for (int k = 0; k < 1000; ++k)
        openvrx_agc_acc_add(&acc, &p.plan, pads_top(p.lanes, p.n, 4, k & 1 ? 3 : 1, k & 1 ? -2 : -4));
    CHECK(fabsf(openvrx_agc_mean_steps(&acc, 0) - 2.5f) < 1e-4f);    // (3.5 + 1.5) / 2
    CHECK(fabsf(openvrx_agc_mean_steps(&acc, 1) + 2.5f) < 1e-4f);    // (-1.5 - 3.5) / 2
    CHECK(openvrx_agc_neg_share(&acc, 0) == 0.0f && openvrx_agc_neg_share(&acc, 1) == 1.0f);
    CHECK(fabsf(openvrx_agc_rms_ac_steps(&acc) - 1.0f) < 1e-3f);     // +-1 step around each mean
    CHECK(openvrx_agc_rms_steps(&acc) > 2.6f);                        // the DC counted as signal

    // Rows: claim from the nearest claimed row, limits around the vendor code.
    openvrx_dc_rows_t rows;
    openvrx_dc_rows_reset(&rows);
    int trim[2], applied[2];
    bool limited[2];
    const int base[2] = {256, 20};
    int st[2] = {5, -3};
    openvrx_dc_rows_step(&rows, 70, base, st, applied, limited);
    CHECK(applied[0] == 5 && applied[1] == -3 && !limited[0] && !limited[1]);
    openvrx_dc_rows_claim(&rows, 74, trim);
    CHECK(trim[0] == 5 && trim[1] == -3 && rows.have[74]);
    st[0] = 200;
    st[1] = -200;
    openvrx_dc_rows_step(&rows, 70, base, st, applied, limited);
    CHECK(rows.trim[70][0] == OPENVRX_DC_TRIM_MAX && rows.trim[70][1] == -20 && limited[0] && limited[1]);
    CHECK(openvrx_dc_code(20, -40) == 0 && openvrx_dc_code(500, 40) == OPENVRX_DC_CODE_MAX);
    openvrx_dc_rows_claim(&rows, 200, trim);
    CHECK(trim[0] == 0 && trim[1] == 0);

    openvrx_dc_servo_t s;
    float worst;

    // Crash flight: I +216, Q +136 of 512, noise about one step, an FM
    // carrier at the gain loop's level (RMS 0.55 of full scale per axis).
    // Within 3 s the DC is under a quarter step and the clipping falls to
    // what the carrier's own peaks cost.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 216; p.dc0[1] = 136; p.slope[0] = 12; p.slope[1] = 10; p.noise = 60; p.carrier = 300;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    const float clip_before = dc_clip_now(&p, &rows, 70);
    CHECK(clip_before > 0.10f);
    worst = dc_run(&p, &s, &rows, 70, 1, 30, 10, 0.0f);
    // Bound: the deadband, half a code plus two (inflated) standard errors.
    CHECK(worst < 20.0f);
    const float clip_after = dc_clip_now(&p, &rows, 70);
    CHECK(clip_after < 0.25f * clip_before);
    if (getenv("DC_TRACE")) printf("crash flight clip %.3f -> %.3f\n", (double)clip_before, (double)clip_after);
    // Converged: little stepping on measurement noise over the next 5 s.
    const uint32_t steps_before = s.steps;
    worst = dc_run(&p, &s, &rows, 70, 1, 50, 50, 0.0f);
    CHECK(worst < 20.0f && s.steps - steps_before <= 5u);
    CHECK(s.flips == 0 && s.pol[0] == 1 && s.pol[1] == 1);

    // Receiver noise under one step (no carrier, low gain): the IQ is one
    // fixed point; the sign-bit bracket finds zero within one code.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 150; p.dc0[1] = -100; p.slope[0] = 12; p.slope[1] = 12; p.noise = 3;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 40, 1, 40, 10, 0.0f);
    CHECK(worst <= 12.0f + 3.0f);
    CHECK(s.flips == 0);

    // Inverted DAC polarity: found and corrected, still converges.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 200; p.dc0[1] = -120; p.slope[0] = -12; p.slope[1] = -12; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 70, 1, 80, 10, 0.0f);
    CHECK(worst < 20.0f);
    CHECK(s.pol[0] == -1 && s.pol[1] == -1);

    // Inverted polarity with a fixed point (no linear zone to learn from).
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 120; p.dc0[1] = 90; p.slope[0] = -10; p.slope[1] = -10; p.noise = 3;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 40, 1, 80, 10, 0.0f);
    CHECK(worst <= 10.0f + 3.0f);

    // One code moves a lot (45 ADC units): no hunting, within a code.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 216; p.dc0[1] = -150; p.slope[0] = 45; p.slope[1] = 45; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 70, 1, 40, 20, 0.0f);
    CHECK(worst < 45.0f * 0.6f + 6.0f);

    // One code moves little (3 ADC units): 50+ codes needed, within 6 s.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 180; p.dc0[1] = 150; p.slope[0] = 3; p.slope[1] = 3; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 70, 1, 60, 10, 0.0f);
    CHECK(worst < 12.0f);

    // Beyond the DAC's reach (96 codes x 3 = 288 < 450): the trim parks at
    // its limit on the right side and stays there.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 450; p.dc0[1] = 0; p.slope[0] = 3; p.slope[1] = 3; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    (void)dc_run(&p, &s, &rows, 70, 1, 100, 1, 0.0f);
    CHECK(rows.trim[70][0] == -OPENVRX_DC_TRIM_MAX);
    CHECK(s.limits > 0);

    // I/Q cross-coupling of 15 %, and a DC that wanders (C5VRX: 1-2 cells
    // over seconds): tracked within a quarter step.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.dc0[0] = 120; p.dc0[1] = -90; p.slope[0] = 12; p.slope[1] = 12; p.cross = 0.15f; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 70, 1, 100, 70, 2.0f);
    CHECK(worst < 20.0f);

    // Row hops (the gain loop moving): each row converges, a new row starts
    // from its neighbour's trim, and a row change forgets the last step.
    plant_init(&p, OPENVRX_PRESET_4X4TOP);
    p.slope[0] = 12; p.slope[1] = 12; p.noise = 60;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    for (unsigned round = 0; round < 12; ++round)
        for (unsigned r = 66; r <= 70; ++r) {
            p.dc0[0] = 200.0f + 4.0f * (float)(r - 66);   // common shift, small per-row spread
            p.dc0[1] = 130.0f - 3.0f * (float)(r - 66);
            (void)dc_run(&p, &s, &rows, r, round * 8u + r, 3, 1, 0.0f);
        }
    for (unsigned r = 66; r <= 70; ++r) {
        p.dc0[0] = 200.0f + 4.0f * (float)(r - 66);
        p.dc0[1] = 130.0f - 3.0f * (float)(r - 66);
        openvrx_dc_rows_claim(&rows, r, trim);
        CHECK(fabsf(plant_dc(&p, trim, 0)) < 20.0f && fabsf(plant_dc(&p, trim, 1)) < 20.0f);
    }
    openvrx_dc_rows_claim(&rows, 71, trim);   // never visited: the neighbour's trim
    CHECK(trim[0] == rows.trim[70][0] && trim[1] == rows.trim[70][1]);
    CHECK(s.flips == 0);

    // 8x8 (step 4 ADC units): same crash-flight offsets.
    plant_init(&p, OPENVRX_PRESET_8X8);
    p.dc0[0] = 216; p.dc0[1] = 136; p.slope[0] = 12; p.slope[1] = 10; p.noise = 60; p.carrier = 150;
    openvrx_dc_servo_init(&s);
    openvrx_dc_rows_reset(&rows);
    worst = dc_run(&p, &s, &rows, 70, 1, 30, 10, 0.0f);
    CHECK(worst < 20.0f);

    // Command.
    char line[32] = "dc off";
    openvrx_cmd_t cmd;
    openvrx_err_t err;
    const char *word;
    CHECK(openvrx_cmd_parse(line, &cmd, &err, &word) && cmd.kind == OPENVRX_CMD_DC && !cmd.a.dc_on);
    strcpy(line, "dc on");
    CHECK(openvrx_cmd_parse(line, &cmd, &err, &word) && cmd.a.dc_on);
    strcpy(line, "dc");
    CHECK(!openvrx_cmd_parse(line, &cmd, &err, &word));
}

// ---------------------------------------------------------------- agc fades (C5-4)

// FM carrier plus receiver noise ahead of the gain: both scale with the
// gain's dB (db_per_index around G60). At sig_db 0 and G60 the carrier is
// 400 ADC units (RMS 283, about the loop's 0.55 target on 4x4top).
static void fade_plant(dc_plant_t *p, float sig_db, unsigned gain, float db_per_index)
{
    const float g = powf(10.0f, db_per_index * ((float)gain - 60.0f) / 20.0f);
    p->carrier = 400.0f * powf(10.0f, sig_db / 20.0f) * g;
    p->noise = 6.0f * g;
}

typedef struct {
    unsigned gain;
    unsigned writes;       // gain changes
    unsigned clip_ticks;   // 5 ms ticks with 5 % or more clipped
    int clear_ms;          // ms from the run's start to the first tick under 5 % (-1: none yet)
} fade_result_t;

// Share of 5 ms ticks whose batch an interference burst saturates.
static float s_burst_rate;

// Runs `ms` of 5 ms ticks at sig_db from state r; fast = the new loop.
static void fade_run(dc_plant_t *p, openvrx_agc_ctl_t *c, fade_result_t *r, uint32_t *t, uint32_t ms, float sig_db,
                     float db_per_index, bool fast)
{
    static const int trim0[2] = {0, 0};
    const uint32_t start = *t;
    r->clear_ms = -1;
    for (uint32_t end = *t + ms; *t < end; *t += OPENVRX_AGC_FAST_MS) {
        fade_plant(p, sig_db, r->gain, db_per_index);
        if (s_burst_rate > 0.0f && frand() < s_burst_rate) p->carrier = 3000.0f;   // a packet, rails both axes
        openvrx_agc_acc_t acc;
        unsigned next = r->gain;
        if (*t % OPENVRX_AGC_PERIOD_MS == 0) {
            plant_measure_n(p, trim0, &acc, OPENVRX_AGC_READS);
            const float clip = openvrx_agc_clip_share(&acc), rms = openvrx_agc_rms_ac_steps(&acc);
            next = fast ? openvrx_agc_slow(c, *t, r->gain, 0, 83, 4, clip, rms)
                        : openvrx_agc_next(r->gain, 0, 83, 4, clip, rms);
        } else {
            plant_measure_n(p, trim0, &acc, OPENVRX_AGC_FAST_READS);
            if (fast) next = openvrx_agc_fast(c, *t, r->gain, 0, openvrx_agc_clip_share(&acc));
        }
        if (openvrx_agc_clip_share(&acc) >= 0.05f) ++r->clip_ticks;
        else if (r->clear_ms < 0) r->clear_ms = (int)(*t - start);
        if (getenv("AGC_TICKS") && fast && sig_db > 0.0f && *t < 4300)
            printf("t %u gain %u clip %.3f rms %.2f next %u\n", *t, r->gain, (double)openvrx_agc_clip_share(&acc),
                   (double)openvrx_agc_rms_ac_steps(&acc), next);
        if (next != r->gain) {
            ++r->writes;
            r->gain = next;
        }
    }
}

static void test_agc_fades(void)
{
    // Pure rules.
    openvrx_agc_ctl_t c;
    openvrx_agc_ctl_init(&c);
    CHECK(openvrx_agc_fast(&c, 1000, 70, 0, 0.05f) == 70);                        // under the threshold
    // Three hot batches in a row before the first attack.
    CHECK(openvrx_agc_fast(&c, 1005, 70, 0, 0.50f) == 70);
    CHECK(openvrx_agc_fast(&c, 1010, 70, 0, 0.50f) == 70);
    CHECK(openvrx_agc_fast(&c, 1015, 70, 0, 0.50f) == 70 - OPENVRX_AGC_ATTACK_STEP);
    CHECK(openvrx_agc_fast(&c, 1020, 64, 0, 0.50f) == 64);                        // within the gap
    CHECK(openvrx_agc_fast(&c, 1025, 64, 0, 0.50f) == 64 - OPENVRX_AGC_ATTACK_STEP);
    CHECK(openvrx_agc_fast(&c, 1100, 3, 0, 0.90f) == 0 && openvrx_agc_fast(&c, 1200, 0, 0, 0.90f) == 0);
    CHECK(c.attacks == 3);
    // Interference bursts (board 2026-11-04, no VTX): hot batches that do not
    // last three in a row never attack.
    {
        openvrx_agc_ctl_t b;
        openvrx_agc_ctl_init(&b);
        static const float clips[] = {0.9f, 0.0f, 0.9f, 0.9f, 0.0f, 0.6f, 0.0f, 0.9f, 0.9f, 0.02f, 0.9f};
        for (unsigned k = 0; k < sizeof(clips) / sizeof(clips[0]); ++k)
            CHECK(openvrx_agc_fast(&b, 2000 + 5u * k, 83, 0, clips[k]) == 83);
        CHECK(b.attacks == 0);
    }
    // Raise held off after an attack, lowering never.
    CHECK(openvrx_agc_slow(&c, 1300, 50, 0, 83, 4, 0.0f, 1.0f) == 50);
    CHECK(openvrx_agc_slow(&c, 1300, 50, 0, 83, 4, 0.9f, 7.0f) < 50);
    CHECK(openvrx_agc_slow(&c, 1200 + OPENVRX_AGC_RAISE_HOLD_MS, 50, 0, 83, 4, 0.0f, 1.0f) > 50);
    // Wrap-safe: an attack just before the uint32 ms wrap still holds raises after it.
    openvrx_agc_ctl_init(&c);
    (void)openvrx_agc_fast(&c, 0xFFFFFFE6u, 70, 0, 0.5f);
    (void)openvrx_agc_fast(&c, 0xFFFFFFEBu, 70, 0, 0.5f);
    CHECK(openvrx_agc_fast(&c, 0xFFFFFFF0u, 70, 0, 0.5f) < 70);
    CHECK(openvrx_agc_slow(&c, 0x00000010u, 50, 0, 83, 4, 0.0f, 1.0f) == 50);

    // Simulated fade recovery: settle weak (-20 dB), then +25 dB at once.
    static const float dpi[2] = {0.5f, 2.0f};   // dB per index: the assumption, and much steeper
    for (unsigned k = 0; k < 2; ++k) {
        fade_result_t r_old = {.gain = 60}, r_new = {.gain = 60};
        dc_plant_t p;
        plant_init(&p, OPENVRX_PRESET_4X4TOP);
        uint32_t t = 0;
        openvrx_agc_ctl_init(&c);
        fade_run(&p, &c, &r_old, &t, 4000, -20.0f, dpi[k], false);
        t = 0;
        fade_run(&p, &c, &r_new, &t, 4000, -20.0f, dpi[k], true);
        if (getenv("AGC_TRACE"))
            printf("settle -20 dB, %.1f dB/index: gain old %u new %u, writes old %u new %u, attacks %u\n",
                   (double)dpi[k], r_old.gain, r_new.gain, r_old.writes, r_new.writes, c.attacks);
        // Steady weak signal. With the assumed 0.5 dB per index the attack
        // never fires. At 2 dB per index the old loop hunts (4 indexes = 8 dB
        // steps against a 1.5 dB deadband: 40 writes in 4 s); the new one
        // overshoots once, the attack catches it and the raise hold-off ends
        // the hunting.
        if (k == 0) CHECK(c.attacks == 0);
        else CHECK(r_new.writes * 4 <= r_old.writes);
        const unsigned settled_writes = r_new.writes;
        r_old.writes = r_new.writes = r_old.clip_ticks = r_new.clip_ticks = 0;
        uint32_t t_old = 4000, t_new = 4000;
        fade_run(&p, &c, &r_old, &t_old, 1000, 5.0f, dpi[k], false);
        fade_run(&p, &c, &r_new, &t_new, 1000, 5.0f, dpi[k], true);
        const unsigned old_ms = r_old.clip_ticks * OPENVRX_AGC_FAST_MS, new_ms = r_new.clip_ticks * OPENVRX_AGC_FAST_MS;
        if (getenv("AGC_TRACE"))
            printf("fade +25 dB, %.1f dB/index: recovered old %d ms, new %d ms; clipped in 1 s old %u ms, "
                   "new %u ms; writes old %u new %u (attacks %u, settle writes %u)\n",
                   (double)dpi[k], r_old.clear_ms, r_new.clear_ms, old_ms, new_ms, r_old.writes, r_new.writes,
                   c.attacks, settled_writes);
        // Fade response (C5-4): 25 dB is 4 to 9 attacks 10 ms apart.
        CHECK(r_new.clear_ms >= 0 && r_new.clear_ms <= 60);
        CHECK(r_old.clear_ms < 0 || r_old.clear_ms >= 4 * r_new.clear_ms);
        // Clipped time over the second, including any later overshoot of
        // the 100 ms loop's raises when the dB per index is steeper than
        // assumed (C5-3; each overshoot costs the 10 ms confirmation plus
        // the attack): at most a third of the old loop's.
        CHECK(new_ms * 3 <= old_ms);
        CHECK(r_new.writes <= 15);
        // Then steady at the new level: no pumping (at most a few writes in 3 s).
        r_new.writes = 0;
        fade_run(&p, &c, &r_new, &t_new, 3000, 5.0f, dpi[k], true);
        CHECK(r_new.writes <= 4);
    }

    // Interference bursts on a weak steady signal (board: no VTX, the gain
    // at the top). With one hot batch enough the board attacked about 2
    // times a second, so about 1 % of its batches were hot; here 2 %
    // (4 a second) are railed by a burst. Three in a row by chance is
    // 8e-6 per tick: at most one in 5 s.
    {
        fade_result_t r = {.gain = 60};
        dc_plant_t p;
        plant_init(&p, OPENVRX_PRESET_4X4TOP);
        uint32_t t = 0;
        openvrx_agc_ctl_init(&c);
        fade_run(&p, &c, &r, &t, 3000, -20.0f, 0.5f, true);   // settle without bursts
        const unsigned settled = c.attacks;
        s_burst_rate = 0.02f;
        r.writes = 0;
        fade_run(&p, &c, &r, &t, 5000, -20.0f, 0.5f, true);
        if (getenv("AGC_TRACE"))
            printf("bursts on a weak signal: attacks %u, writes %u in 5 s, gain %u\n", c.attacks - settled, r.writes,
                   r.gain);
        CHECK(c.attacks - settled <= 1);
        // Fade recovery still works with the bursts on.
        fade_result_t f = r;
        f.clip_ticks = 0;
        fade_run(&p, &c, &f, &t, 1000, 5.0f, 0.5f, true);
        CHECK(f.clear_ms >= 0 && f.clear_ms <= 80);
        s_burst_rate = 0.0f;
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--tables") == 0) {
        print_tables();
        return 0;
    }
    test_lanes();
    test_boards();
    test_cmd();
    test_selftest();
    test_slip();
    test_settings_gain();
    test_agc();
    test_dc();
    test_agc_fades();
    printf("%u checks, %u failures\n", s_checks, s_failures);
    return s_failures ? 1 : 0;
}
