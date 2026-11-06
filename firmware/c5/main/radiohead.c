// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: device state and console command execution
// (docs/PROTOCOL.md section 1). Every handler ends with exactly one
// console_reply().
#include "radiohead.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "agc.h"
#include "boot_guard.h"
#include "console.h"
#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lanes_hw.h"
#include "openvrx_boards.h"
#include "openvrx_cmd.h"
#include "openvrx_sclk_logic.h"
#include "rf.h"
#include "sclk.h"
#include "sdkconfig.h"
#include "settings.h"

#define PROTO_VERSION 1

static openvrx_board_map_t s_custom;
static bool s_custom_valid;
static char s_custom_why[48];
static openvrx_board_id_t s_board = OPENVRX_BOARD_XIAO;
static unsigned s_drive_lanes = 1, s_drive_clk = 2;

// ---------------------------------------------------------------- helpers

static void reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void reply(const char *fmt, ...)
{
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    console_reply(line);
}

static void reply_err(openvrx_cmd_kind_t kind, openvrx_err_code_t code, const char *text)
{
    openvrx_err_t e = {.code = code};
    snprintf(e.text, sizeof(e.text), "%s", text);
    char line[160];
    openvrx_format_err(line, sizeof(line), openvrx_cmd_word(kind), &e);
    console_reply(line);
}

// Maps a driver error to an ERR reply.
static void reply_esp_err(openvrx_cmd_kind_t kind, esp_err_t err)
{
    if (err == ESP_ERR_INVALID_STATE) reply_err(kind, OPENVRX_ERR_STATE, "radio not started");
    else if (err == ESP_ERR_NOT_SUPPORTED) reply_err(kind, OPENVRX_ERR_UNSUPPORTED, "PHY not pinned");
    else if (err == ESP_ERR_INVALID_RESPONSE) reply_err(kind, OPENVRX_ERR_HW, "phy readback failed");
    else reply_err(kind, OPENVRX_ERR_HW, esp_err_to_name(err));
}

static void build_custom_map(void)
{
    s_custom.name = "custom";
    s_custom.clk = (int8_t)CONFIG_OPENVRX_CUSTOM_CLK;
    const int w16[16] = {
        CONFIG_OPENVRX_CUSTOM_PB0, CONFIG_OPENVRX_CUSTOM_PB1, CONFIG_OPENVRX_CUSTOM_PB2, CONFIG_OPENVRX_CUSTOM_PB3,
        CONFIG_OPENVRX_CUSTOM_PB4, CONFIG_OPENVRX_CUSTOM_PB5, CONFIG_OPENVRX_CUSTOM_PB6, CONFIG_OPENVRX_CUSTOM_PB7,
        CONFIG_OPENVRX_CUSTOM_PD0, CONFIG_OPENVRX_CUSTOM_PD1, CONFIG_OPENVRX_CUSTOM_PD2, CONFIG_OPENVRX_CUSTOM_PD3,
        CONFIG_OPENVRX_CUSTOM_PD4, CONFIG_OPENVRX_CUSTOM_PD5, CONFIG_OPENVRX_CUSTOM_PD6, CONFIG_OPENVRX_CUSTOM_PD7,
    };
    const int w8[8] = {
        CONFIG_OPENVRX_CUSTOM8_PB0, CONFIG_OPENVRX_CUSTOM8_PB1, CONFIG_OPENVRX_CUSTOM8_PB2, CONFIG_OPENVRX_CUSTOM8_PB3,
        CONFIG_OPENVRX_CUSTOM8_PB4, CONFIG_OPENVRX_CUSTOM8_PB5, CONFIG_OPENVRX_CUSTOM8_PB6, CONFIG_OPENVRX_CUSTOM8_PB7,
    };
    for (unsigned k = 0; k < 16; ++k) s_custom.w16[k] = (int8_t)w16[k];
    for (unsigned k = 0; k < 8; ++k) s_custom.w8[k] = (int8_t)w8[k];
    s_custom_valid = openvrx_board_validate(&s_custom, s_custom_why, sizeof(s_custom_why));
}

static const openvrx_board_map_t *board_map(openvrx_board_id_t id)
{
    if (id == OPENVRX_BOARD_CUSTOM) return s_custom_valid ? &s_custom : NULL;
    return openvrx_board_builtin(id);
}

static openvrx_preset_t current_preset(void)
{
    return lanes_hw_route()->preset;
}

// Routes a preset on the current board (NONE when it does not fit).
static esp_err_t route_preset(openvrx_preset_t p)
{
    openvrx_route_t r;
    if (openvrx_route(board_map(s_board), p, &r, NULL) != OPENVRX_ROUTE_OK)
        (void)openvrx_route(board_map(s_board), OPENVRX_PRESET_NONE, &r, NULL);
    return lanes_hw_apply(&r, s_drive_lanes);
}

// Comma-separated GPIO list of one component, MSB first.
static void pin_list(char *out, size_t len, char comp)
{
    const openvrx_route_t *r = lanes_hw_route();
    size_t n = 0;
    out[0] = 0;
    for (unsigned k = r->n; k-- > 0;) {
        if (r->lane[k].comp != comp) continue;
        n += (size_t)snprintf(out + n, len - n, "%sG%d", n ? "," : "", r->gpio[k]);
        if (n >= len) break;
    }
    if (!out[0]) snprintf(out, len, "-");
}

static const char *filter_text(int code, char *buf, size_t len)
{
    if (code == OPENVRX_FILTER_CAL) return "cal";
    snprintf(buf, len, "%d", code);
    return buf;
}

// ---------------------------------------------------------------- boot

void radiohead_defaults(openvrx_settings_t *s)
{
    build_custom_map();
#if CONFIG_OPENVRX_DEFAULT_BOARD_ZERO
    openvrx_board_id_t b = OPENVRX_BOARD_ZERO;
#elif CONFIG_OPENVRX_DEFAULT_BOARD_CUSTOM
    openvrx_board_id_t b = s_custom_valid ? OPENVRX_BOARD_CUSTOM : OPENVRX_BOARD_XIAO;
#else
    openvrx_board_id_t b = OPENVRX_BOARD_XIAO;
#endif
    openvrx_settings_default(s, b, openvrx_board_default_preset(board_map(b)));
}

void radiohead_boot(const openvrx_settings_t *s)
{
    build_custom_map();
    if (!s_custom_valid && s->board == OPENVRX_BOARD_CUSTOM) console_log("custom pin map invalid: %s", s_custom_why);
    s_board = board_map((openvrx_board_id_t)s->board) ? (openvrx_board_id_t)s->board : OPENVRX_BOARD_XIAO;
    s_drive_lanes = s->drive_lanes;
    s_drive_clk = s->drive_clk;
    boot_guard_stage(BOOT_STAGE_LANES_INIT);
    esp_err_t err = lanes_hw_init();
    if (err != ESP_OK) console_log("self-test timer: %s", esp_err_to_name(err));
    boot_guard_stage(BOOT_STAGE_CLOCK);
    err = sclk_start(s->div, board_map(s_board)->clk, s_drive_clk);
    if (err != ESP_OK) console_log("sample clock: %s", esp_err_to_name(err));
    boot_guard_stage(BOOT_STAGE_ROUTE);
    err = route_preset((openvrx_preset_t)s->preset);
    if (err != ESP_OK) console_log("lanes: %s", esp_err_to_name(err));
    boot_guard_stage(BOOT_STAGE_RADIO);
    const rf_config_t rc = {
        .mhz = s->mhz,
        .off_khz = s->off_khz,
        .gain = s->gain,
        .p11 = s->p11 != 0,
        .filter_code = s->filter_code,
        .skirt = s->skirt,
        .bw20 = s->bw_mhz == 20,
        .dc = s->dc != 0,
    };
    err = rf_start(&rc);
    if (err != ESP_OK) console_log("radio start: %s", esp_err_to_name(err));
    err = agc_start(s->agc != 0);
    if (err != ESP_OK) console_log("agc: %s", esp_err_to_name(err));
}

// ---------------------------------------------------------------- commands

static void cmd_ver(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    reply("OK ver fw=%s proto=%d board=%s idf=%s chip=esp32c5 rev=%d.%d", esp_app_get_description()->version,
          PROTO_VERSION, openvrx_board_name(s_board), esp_get_idf_version(), chip.revision / 100, chip.revision % 100);
}

static void cmd_status(void)
{
    const rf_config_t c = rf_config();
    char fbuf[8];
    const openvrx_selftest_mode_t st = lanes_hw_selftest_mode();
    unsigned clip, rms;
    bool measurable;
    agc_last(&clip, &rms, &measurable);
    char level[40];
    if (measurable) snprintf(level, sizeof(level), "clip=%u.%u rms=%u.%02u", clip / 10, clip % 10, rms / 100, rms % 100);
    else snprintf(level, sizeof(level), "clip=- rms=-");
    // DC: measured mean (ADC units, full scale 512) and the held DAC pair
    // with its trim from the vendor code.
    rf_dc_info_t d;
    rf_dc_info(&d);
    int dc_adc[2];
    agc_last_dc(dc_adc);
    char dc[80];
    int len = measurable ? snprintf(dc, sizeof(dc), "i_dc=%d q_dc=%d", dc_adc[0], dc_adc[1])
                         : snprintf(dc, sizeof(dc), "i_dc=- q_dc=-");
    if (len < 0) len = 0;
    if (d.held)
        snprintf(dc + len, sizeof(dc) - (size_t)len, " dco=%d/%d trim=%d/%d", d.code[0], d.code[1], d.trim[0], d.trim[1]);
    else
        snprintf(dc + len, sizeof(dc) - (size_t)len, " dco=%s trim=-", d.failed ? "error" : "-");
    reply("OK status mhz=%u off_khz=%d gain=%u agc=%d %s atk=%u dc=%d %s clk=%lu clk_on=%d lanes=%s p11=%d filter=%s skirt=%u bw=%u "
          "selftest=%s up_ms=%llu slips=%lu phy=%s board=%s hz=%lu boot=%s",
          c.mhz, c.off_khz, c.gain, agc_enabled() ? 1 : 0, level, agc_attacks(), c.dc ? 1 : 0, dc, (unsigned long)sclk_div(), sclk_on() ? 1 : 0, openvrx_preset_name(current_preset()),
          c.p11 ? 1 : 0, filter_text(c.filter_code, fbuf, sizeof(fbuf)), c.skirt, c.bw20 ? 20u : 40u,
          lanes_hw_selftest_name(st), (unsigned long long)(esp_timer_get_time() / 1000), (unsigned long)sclk_slips(),
          rf_phy_state_name(rf_phy_state()), openvrx_board_name(s_board), (unsigned long)openvrx_clock_hz(sclk_div()),
          boot_guard_stalled() == BOOT_STAGE_NONE ? "ok" : boot_stage_name(boot_guard_stalled()));
}

static void cmd_board(openvrx_board_id_t id)
{
    const openvrx_board_map_t *m = board_map(id);
    if (!m) {
        char text[96];
        snprintf(text, sizeof(text), "custom map invalid: %s", s_custom_why);
        reply_err(OPENVRX_CMD_BOARD, OPENVRX_ERR_STATE, text);
        return;
    }
    const openvrx_preset_t want = current_preset();
    s_board = id;
    // Clock first: the old clock pad is released before the lanes claim it.
    esp_err_t err = sclk_set_pin(m->clk);
    if (err == ESP_OK) err = route_preset(want);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_BOARD, err);
        return;
    }
    reply("OK board name=%s pins=%u lanes=%s", m->name, openvrx_board_pin_count(m), openvrx_preset_name(current_preset()));
}

static void cmd_pins(void)
{
    char q[96], i[96];
    pin_list(q, sizeof(q), 'Q');
    pin_list(i, sizeof(i), 'I');
    reply("OK pins board=%s lanes=%s clk=G%d q=%s i=%s", openvrx_board_name(s_board),
          openvrx_preset_name(current_preset()), sclk_pin(), q, i);
}

static void cmd_lanes(openvrx_preset_t p)
{
    if (!openvrx_preset_is_iq(p)) {
        reply_err(OPENVRX_CMD_LANES, OPENVRX_ERR_UNSUPPORTED, "angle8 needs RX+BitScrambler path (HARDWARE.md)");
        return;
    }
    openvrx_route_t r;
    int missing = -1;
    if (openvrx_route(board_map(s_board), p, &r, &missing) != OPENVRX_ROUTE_OK) {
        char text[64];
        snprintf(text, sizeof(text), "board %s has no pin for %s", openvrx_board_name(s_board),
                 missing >= 0 ? openvrx_fx2_pin_name((unsigned)missing) : "a lane");
        reply_err(OPENVRX_CMD_LANES, OPENVRX_ERR_RANGE, text);
        return;
    }
    const esp_err_t err = lanes_hw_apply(&r, s_drive_lanes);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_LANES, err);
        return;
    }
    reply("OK lanes preset=%s data_pins=%u word_bytes=%u", openvrx_preset_name(p), openvrx_preset_data_pins(p),
          openvrx_preset_word_bytes(p));
}

static void cmd_tune(int mhz, int off_khz)
{
    const esp_err_t err = rf_tune((uint16_t)mhz, off_khz);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_TUNE, err);
        return;
    }
    const rf_config_t c = rf_config();
    reply("OK tune mhz=%u off_khz=%d", c.mhz, c.off_khz);
}

static void cmd_gain(unsigned gain)
{
    if (rf_started() && gain > rf_gain_max()) {
        char text[48];
        snprintf(text, sizeof(text), "index above table max %u", rf_gain_max());
        reply_err(OPENVRX_CMD_GAIN, OPENVRX_ERR_RANGE, text);
        return;
    }
    const esp_err_t err = rf_set_gain((uint8_t)gain);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_GAIN, err);
        return;
    }
    reply("OK gain index=%u", gain);
}

static void cmd_gains(void)
{
    if (!rf_started()) {
        reply_err(OPENVRX_CMD_GAINS, OPENVRX_ERR_STATE, "radio not started");
        return;
    }
    uint8_t starts[16];
    const unsigned n = rf_gain_stage_starts(starts, sizeof(starts));
    char list[96];
    size_t len = 0;
    list[0] = 0;
    for (unsigned k = 0; k < n && len < sizeof(list); ++k)
        len += (size_t)snprintf(list + len, sizeof(list) - len, "%s%u", k ? "," : "", starts[k]);
    reply("OK gains max=%u stages=%s", rf_gain_max(), list);
}

static void cmd_clk(openvrx_clk_action_t action, unsigned div)
{
    esp_err_t err;
    if (action == OPENVRX_CLK_DIV) {
        err = sclk_set_div(div);
        if (err != ESP_OK) {
            reply_esp_err(OPENVRX_CMD_CLK, err);
            return;
        }
        reply("OK clk div=%u hz=%lu locked=%d", div, (unsigned long)openvrx_clock_hz(div),
              openvrx_div_bus_locked(div) ? 1 : 0);
        return;
    }
    err = sclk_gate(action == OPENVRX_CLK_ON);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_CLK, err);
        return;
    }
    reply("OK clk on=%d", sclk_on() ? 1 : 0);
}

static void cmd_slip(unsigned n)
{
    if (!sclk_on()) {
        reply_err(OPENVRX_CMD_SLIP, OPENVRX_ERR_STATE, "clock is off");
        return;
    }
    const uint32_t before = sclk_slips();
    const esp_err_t err = sclk_slip(n);
    if (err == ESP_ERR_TIMEOUT) {
        reply_err(OPENVRX_CMD_SLIP, OPENVRX_ERR_HW, "clock pad not toggling");
        return;
    }
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_SLIP, err);
        return;
    }
    const uint32_t total = sclk_slips();
    reply("OK slip n=%lu total=%lu rel=%lu", (unsigned long)(total - before), (unsigned long)total,
          (unsigned long)(total % OPENVRX_BUS_TICKS));
}

static void cmd_p11(bool on)
{
    const esp_err_t err = rf_set_p11(on);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_P11, err);
        return;
    }
    reply("OK p11 on=%d", on ? 1 : 0);
}

static void cmd_filter(int code, int skirt)
{
    const esp_err_t err = rf_set_filter(code, skirt);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_FILTER, err);
        return;
    }
    char fbuf[8];
    const rf_config_t c = rf_config();
    reply("OK filter code=%s skirt=%u", filter_text(c.filter_code, fbuf, sizeof(fbuf)), c.skirt);
}

static void cmd_bw(unsigned mhz)
{
    const esp_err_t err = rf_set_bw20(mhz == 20);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_BW, err);
        return;
    }
    reply("OK bw mhz=%u", mhz);
}

static void cmd_drive(unsigned lanes, int clk)
{
    esp_err_t err = lanes_hw_set_drive(lanes);
    if (err == ESP_OK && clk >= 0) err = sclk_set_drive((unsigned)clk);
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_DRIVE, err);
        return;
    }
    s_drive_lanes = lanes;
    if (clk >= 0) s_drive_clk = (unsigned)clk;
    reply("OK drive lanes=%u clk=%u", s_drive_lanes, s_drive_clk);
}

static void cmd_selftest(openvrx_selftest_mode_t mode, uint32_t value, uint32_t phase_us)
{
    const unsigned n = lanes_hw_route()->n;
    if (mode != OPENVRX_ST_OFF && n == 0) {
        reply_err(OPENVRX_CMD_SELFTEST, OPENVRX_ERR_STATE, "no lanes routed");
        return;
    }
    const esp_err_t err = lanes_hw_selftest(mode, value, phase_us);
    if (err == ESP_ERR_INVALID_ARG) {
        reply_err(OPENVRX_CMD_SELFTEST, OPENVRX_ERR_RANGE, "value wider than the lanes");
        return;
    }
    if (err != ESP_OK) {
        reply_esp_err(OPENVRX_CMD_SELFTEST, err);
        return;
    }
    if (mode == OPENVRX_ST_OFF) reply("OK selftest mode=off");
    else if (mode == OPENVRX_ST_CONST) reply("OK selftest mode=const lanes=%u value=%lx", n, (unsigned long)value);
    else reply("OK selftest mode=%s lanes=%u phase_us=%lu", lanes_hw_selftest_name(mode), n, (unsigned long)phase_us);
}

static void cmd_dc(bool on)
{
    const esp_err_t err = rf_dc_set_enabled(on);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        reply_err(OPENVRX_CMD_DC, OPENVRX_ERR_UNSUPPORTED, "needs the pinned PHY");
        return;
    }
    rf_dc_info_t d;
    rf_dc_info(&d);
    if (err != ESP_OK) {
        reply_err(OPENVRX_CMD_DC, OPENVRX_ERR_HW, "hold did not read back");
        return;
    }
    if (d.held) reply("OK dc on=%d held=1 dco=%d/%d", on ? 1 : 0, d.code[0], d.code[1]);
    else reply("OK dc on=%d held=0", on ? 1 : 0);
}

static void cmd_save(void)
{
    const rf_config_t c = rf_config();
    openvrx_settings_t s;
    openvrx_settings_default(&s, s_board, current_preset());
    s.p11 = c.p11 ? 1 : 0;
    s.div = (uint16_t)sclk_div();
    s.mhz = c.mhz;
    s.off_khz = c.off_khz;
    s.gain = c.gain;
    s.filter_code = c.filter_code;
    s.skirt = c.skirt;
    s.bw_mhz = c.bw20 ? 20 : 40;
    s.drive_lanes = (uint8_t)s_drive_lanes;
    s.drive_clk = (uint8_t)s_drive_clk;
    s.agc = agc_enabled() ? 1 : 0;
    s.dc = c.dc ? 1 : 0;
    const esp_err_t err = settings_store(&s);
    if (err != ESP_OK) {
        reply_err(OPENVRX_CMD_SAVE, OPENVRX_ERR_HW, esp_err_to_name(err));
        return;
    }
    reply("OK save");
}

void radiohead_command(char *line)
{
    openvrx_cmd_t cmd;
    openvrx_err_t err;
    const char *word;
    if (!openvrx_cmd_parse(line, &cmd, &err, &word)) {
        char out[160];
        openvrx_format_err(out, sizeof(out), word, &err);
        console_reply(out);
        return;
    }
    switch (cmd.kind) {
    case OPENVRX_CMD_VER: cmd_ver(); break;
    case OPENVRX_CMD_STATUS: cmd_status(); break;
    case OPENVRX_CMD_BOARD: cmd_board(cmd.a.board); break;
    case OPENVRX_CMD_PINS: cmd_pins(); break;
    case OPENVRX_CMD_LANES: cmd_lanes(cmd.a.preset); break;
    case OPENVRX_CMD_TUNE: cmd_tune(cmd.a.tune.mhz, cmd.a.tune.off_khz); break;
    case OPENVRX_CMD_GAIN: cmd_gain(cmd.a.gain); break;
    case OPENVRX_CMD_GAINS: cmd_gains(); break;
    case OPENVRX_CMD_CLK: cmd_clk(cmd.a.clk.action, cmd.a.clk.div); break;
    case OPENVRX_CMD_SLIP: cmd_slip(cmd.a.slip_n); break;
    case OPENVRX_CMD_P11: cmd_p11(cmd.a.p11_on); break;
    case OPENVRX_CMD_FILTER: cmd_filter(cmd.a.filter.code, cmd.a.filter.skirt); break;
    case OPENVRX_CMD_BW: cmd_bw(cmd.a.bw_mhz); break;
    case OPENVRX_CMD_DRIVE: cmd_drive(cmd.a.drive.lanes, cmd.a.drive.clk); break;
    case OPENVRX_CMD_SELFTEST: cmd_selftest(cmd.a.selftest.mode, cmd.a.selftest.value, cmd.a.selftest.phase_us); break;
    case OPENVRX_CMD_SAVE: cmd_save(); break;
    case OPENVRX_CMD_REBOOT:
        reply("OK reboot");
        console_flush();
        rf_reboot();
        break;
    case OPENVRX_CMD_DL:
        reply("OK dl");
        console_flush();
        rf_reboot_to_download();
        break;
    case OPENVRX_CMD_PING: reply("OK ping"); break;
    case OPENVRX_CMD_AGC:
        agc_set_enabled(cmd.a.agc_on);
        reply("OK agc on=%d", cmd.a.agc_on ? 1 : 0);
        break;
    case OPENVRX_CMD_DC: cmd_dc(cmd.a.dc_on); break;
    default: reply_err(cmd.kind, OPENVRX_ERR_SYNTAX, "unknown command"); break;
    }
}
