// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: console command parser (pure logic).
#include "openvrx_cmd.h"

#include <stdio.h>
#include <string.h>

static const char *const s_words[OPENVRX_CMD_COUNT] = {
    [OPENVRX_CMD_VER] = "ver",
    [OPENVRX_CMD_STATUS] = "status",
    [OPENVRX_CMD_BOARD] = "board",
    [OPENVRX_CMD_PINS] = "pins",
    [OPENVRX_CMD_LANES] = "lanes",
    [OPENVRX_CMD_TUNE] = "tune",
    [OPENVRX_CMD_GAIN] = "gain",
    [OPENVRX_CMD_GAINS] = "gains",
    [OPENVRX_CMD_CLK] = "clk",
    [OPENVRX_CMD_SLIP] = "slip",
    [OPENVRX_CMD_P11] = "p11",
    [OPENVRX_CMD_FILTER] = "filter",
    [OPENVRX_CMD_BW] = "bw",
    [OPENVRX_CMD_DRIVE] = "drive",
    [OPENVRX_CMD_SELFTEST] = "selftest",
    [OPENVRX_CMD_SAVE] = "save",
    [OPENVRX_CMD_REBOOT] = "reboot",
    [OPENVRX_CMD_DL] = "dl",
    [OPENVRX_CMD_PING] = "ping",
};

const char *openvrx_cmd_word(openvrx_cmd_kind_t kind)
{
    return (unsigned)kind < OPENVRX_CMD_COUNT ? s_words[kind] : "?";
}

const char *openvrx_err_code_name(openvrx_err_code_t code)
{
    switch (code) {
    case OPENVRX_ERR_SYNTAX: return "syntax";
    case OPENVRX_ERR_RANGE: return "range";
    case OPENVRX_ERR_STATE: return "state";
    case OPENVRX_ERR_UNSUPPORTED: return "unsupported";
    case OPENVRX_ERR_HW: return "hw";
    default: return "syntax";
    }
}

int openvrx_tokenize(char *line, char *tok[OPENVRX_MAX_TOKENS])
{
    int n = 0;
    char *p = line;
    for (;;) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        if (n == (int)OPENVRX_MAX_TOKENS) return -1;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) *p++ = 0;
    }
    return n;
}

bool openvrx_parse_int(const char *s, long min, long max, long *out)
{
    if (!s || !*s) return false;
    bool neg = false;
    if (*s == '-' || *s == '+') { neg = *s == '-'; ++s; }
    if (!*s) return false;
    long v = 0;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9') return false;
        v = v * 10 + (*s - '0');
        if (v > 100000000L) return false;   // far beyond every range used here
    }
    if (neg) v = -v;
    if (v < min || v > max) return false;
    *out = v;
    return true;
}

bool openvrx_parse_hex(const char *s, uint32_t *out)
{
    if (!s || !*s) return false;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    if (!*s) return false;
    uint32_t v = 0;
    unsigned digits = 0;
    for (; *s; ++s) {
        unsigned d;
        if (*s >= '0' && *s <= '9') d = (unsigned)(*s - '0');
        else if (*s >= 'a' && *s <= 'f') d = (unsigned)(*s - 'a' + 10);
        else if (*s >= 'A' && *s <= 'F') d = (unsigned)(*s - 'A' + 10);
        else return false;
        if (v == 0 && d == 0) continue;   // leading zeros do not count
        if (++digits > 8) return false;
        v = (v << 4) | d;
    }
    *out = v;
    return true;
}

static bool fail(openvrx_err_t *err, openvrx_err_code_t code, const char *text)
{
    err->code = code;
    snprintf(err->text, sizeof(err->text), "%s", text);
    return false;
}

static bool parse_on_off(const char *s, bool *on)
{
    if (strcmp(s, "on") == 0 || strcmp(s, "1") == 0) { *on = true; return true; }
    if (strcmp(s, "off") == 0 || strcmp(s, "0") == 0) { *on = false; return true; }
    return false;
}

// Argument count check: between lo and hi arguments after the command word.
static bool args(int ntok, int lo, int hi, openvrx_err_t *err)
{
    const int n = ntok - 1;
    if (n < lo) return fail(err, OPENVRX_ERR_SYNTAX, "missing argument");
    if (n > hi) return fail(err, OPENVRX_ERR_SYNTAX, "too many arguments");
    return true;
}

bool openvrx_cmd_parse(char *line, openvrx_cmd_t *cmd, openvrx_err_t *err, const char **word_out)
{
    memset(cmd, 0, sizeof(*cmd));
    memset(err, 0, sizeof(*err));
    char *tok[OPENVRX_MAX_TOKENS];
    const int n = openvrx_tokenize(line, tok);
    *word_out = n > 0 ? tok[0] : "?";
    if (n < 0) {
        *word_out = tok[0];   // tokenize filled every slot before giving up
        return fail(err, OPENVRX_ERR_SYNTAX, "too many tokens");
    }
    if (n == 0) return fail(err, OPENVRX_ERR_SYNTAX, "empty line");
    int kind = -1;
    for (unsigned k = 0; k < OPENVRX_CMD_COUNT; ++k)
        if (strcmp(tok[0], s_words[k]) == 0) { kind = (int)k; break; }
    if (kind < 0) return fail(err, OPENVRX_ERR_SYNTAX, "unknown command");
    cmd->kind = (openvrx_cmd_kind_t)kind;
    long v;
    switch (cmd->kind) {
    case OPENVRX_CMD_VER:
    case OPENVRX_CMD_STATUS:
    case OPENVRX_CMD_PINS:
    case OPENVRX_CMD_GAINS:
    case OPENVRX_CMD_SAVE:
    case OPENVRX_CMD_REBOOT:
    case OPENVRX_CMD_DL:
    case OPENVRX_CMD_PING:
        return args(n, 0, 0, err);
    case OPENVRX_CMD_BOARD:
        if (!args(n, 1, 1, err)) return false;
        if (!openvrx_board_parse(tok[1], &cmd->a.board)) return fail(err, OPENVRX_ERR_RANGE, "unknown board");
        return true;
    case OPENVRX_CMD_LANES:
        if (!args(n, 1, 1, err)) return false;
        if (!openvrx_preset_parse(tok[1], &cmd->a.preset)) return fail(err, OPENVRX_ERR_RANGE, "unknown preset");
        return true;
    case OPENVRX_CMD_TUNE:
        if (!args(n, 1, 2, err)) return false;
        if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad MHz");
        if (v < OPENVRX_TUNE_MIN_MHZ || v > OPENVRX_TUNE_MAX_MHZ) return fail(err, OPENVRX_ERR_RANGE, "MHz out of 5180..5945");
        cmd->a.tune.mhz = (int)v;
        cmd->a.tune.off_khz = 0;
        if (n == 3) {
            if (!openvrx_parse_int(tok[2], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad offset");
            if (v < -OPENVRX_OFFSET_MAX_KHZ || v > OPENVRX_OFFSET_MAX_KHZ)
                return fail(err, OPENVRX_ERR_RANGE, "offset out of -2500..2500 kHz");
            cmd->a.tune.off_khz = (int)v;
        }
        return true;
    case OPENVRX_CMD_GAIN:
        if (!args(n, 1, 1, err)) return false;
        if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad index");
        if (v < 0 || v > (long)OPENVRX_GAIN_MAX_INDEX) return fail(err, OPENVRX_ERR_RANGE, "index out of range");
        cmd->a.gain = (unsigned)v;
        return true;
    case OPENVRX_CMD_CLK:
        if (!args(n, 1, 1, err)) return false;
        if (strcmp(tok[1], "on") == 0) { cmd->a.clk.action = OPENVRX_CLK_ON; return true; }
        if (strcmp(tok[1], "off") == 0) { cmd->a.clk.action = OPENVRX_CLK_OFF; return true; }
        if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad divider");
        if (v < (long)OPENVRX_DIV_MIN || v > (long)OPENVRX_DIV_MAX) return fail(err, OPENVRX_ERR_RANGE, "divider out of 6..1023");
        cmd->a.clk.action = OPENVRX_CLK_DIV;
        cmd->a.clk.div = (unsigned)v;
        return true;
    case OPENVRX_CMD_SLIP:
        if (!args(n, 0, 1, err)) return false;
        cmd->a.slip_n = 1;
        if (n == 2) {
            if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad count");
            if (v < 1 || v > (long)OPENVRX_SLIP_MAX) return fail(err, OPENVRX_ERR_RANGE, "count out of 1..64");
            cmd->a.slip_n = (unsigned)v;
        }
        return true;
    case OPENVRX_CMD_P11:
        if (!args(n, 1, 1, err)) return false;
        if (!parse_on_off(tok[1], &cmd->a.p11_on)) return fail(err, OPENVRX_ERR_SYNTAX, "expected on or off");
        return true;
    case OPENVRX_CMD_FILTER:
        if (!args(n, 1, 2, err)) return false;
        if (strcmp(tok[1], "cal") == 0) cmd->a.filter.code = OPENVRX_FILTER_CAL;
        else if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad code");
        else if (v < 0 || v > OPENVRX_FILTER_CODE_MAX) return fail(err, OPENVRX_ERR_RANGE, "code out of 0..63");
        else cmd->a.filter.code = (int)v;
        cmd->a.filter.skirt = 0;
        if (n == 3) {
            if (!openvrx_parse_int(tok[2], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad skirt");
            if (v < 0 || v > OPENVRX_FILTER_SKIRT_MAX) return fail(err, OPENVRX_ERR_RANGE, "skirt out of 0..60");
            cmd->a.filter.skirt = (int)v;
        }
        return true;
    case OPENVRX_CMD_BW:
        if (!args(n, 1, 1, err)) return false;
        if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad bandwidth");
        if (v != 20 && v != 40) return fail(err, OPENVRX_ERR_RANGE, "bandwidth must be 20 or 40");
        cmd->a.bw_mhz = (unsigned)v;
        return true;
    case OPENVRX_CMD_DRIVE:
        if (!args(n, 1, 2, err)) return false;
        if (!openvrx_parse_int(tok[1], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad lane drive");
        if (v < 0 || v > (long)OPENVRX_DRIVE_MAX) return fail(err, OPENVRX_ERR_RANGE, "drive out of 0..3");
        cmd->a.drive.lanes = (unsigned)v;
        cmd->a.drive.clk = -1;
        if (n == 3) {
            if (!openvrx_parse_int(tok[2], -100000, 100000, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad clock drive");
            if (v < 0 || v > (long)OPENVRX_DRIVE_MAX) return fail(err, OPENVRX_ERR_RANGE, "drive out of 0..3");
            cmd->a.drive.clk = (int)v;
        }
        return true;
    case OPENVRX_CMD_SELFTEST: {
        if (!args(n, 1, 3, err)) return false;
        cmd->a.selftest.phase_us = OPENVRX_PHASE_US_DEFAULT;
        cmd->a.selftest.value = 0;
        int next = 2;
        if (strcmp(tok[1], "off") == 0) {
            cmd->a.selftest.mode = OPENVRX_ST_OFF;
            return args(n, 1, 1, err);
        } else if (strcmp(tok[1], "walk") == 0) {
            cmd->a.selftest.mode = OPENVRX_ST_WALK;
        } else if (strcmp(tok[1], "count") == 0) {
            cmd->a.selftest.mode = OPENVRX_ST_COUNT;
        } else if (strcmp(tok[1], "const") == 0) {
            cmd->a.selftest.mode = OPENVRX_ST_CONST;
            if (n < 3) return fail(err, OPENVRX_ERR_SYNTAX, "const needs a HEX value");
            if (n > 3) return fail(err, OPENVRX_ERR_SYNTAX, "too many arguments");
            if (!openvrx_parse_hex(tok[2], &cmd->a.selftest.value)) return fail(err, OPENVRX_ERR_SYNTAX, "bad HEX value");
            return true;
        } else {
            return fail(err, OPENVRX_ERR_SYNTAX, "expected off, walk, const or count");
        }
        if (n > next + 1) return fail(err, OPENVRX_ERR_SYNTAX, "too many arguments");
        if (n == next + 1) {
            if (!openvrx_parse_int(tok[next], -1, 100000000L, &v)) return fail(err, OPENVRX_ERR_SYNTAX, "bad phase_us");
            if (v < (long)OPENVRX_PHASE_US_MIN || v > (long)OPENVRX_PHASE_US_MAX)
                return fail(err, OPENVRX_ERR_RANGE, "phase_us out of 10..10000000");
            cmd->a.selftest.phase_us = (uint32_t)v;
        }
        return true;
    }
    default:
        return fail(err, OPENVRX_ERR_SYNTAX, "unknown command");
    }
}

int openvrx_format_err(char *buf, size_t len, const char *word, const openvrx_err_t *err)
{
    // Echo at most 24 characters of the command word: it may be junk.
    return snprintf(buf, len, "ERR %.24s %s %s", word && *word ? word : "?", openvrx_err_code_name(err->code),
                    err->text[0] ? err->text : "error");
}

void openvrx_linebuf_init(openvrx_linebuf_t *lb)
{
    lb->len = 0;
    lb->overflow = false;
    lb->done = false;
    lb->buf[0] = 0;
}

openvrx_line_status_t openvrx_linebuf_feed(openvrx_linebuf_t *lb, char c)
{
    if (lb->done) openvrx_linebuf_init(lb);
    if (c == '\n') {
        lb->done = true;
        lb->buf[lb->len] = 0;
        return lb->overflow ? OPENVRX_LINE_OVERFLOW : OPENVRX_LINE_READY;
    }
    if (c == '\r' || c == 0) return OPENVRX_LINE_NONE;
    if (lb->len >= OPENVRX_LINE_MAX) {
        lb->overflow = true;
        return OPENVRX_LINE_NONE;
    }
    lb->buf[lb->len++] = c;
    return OPENVRX_LINE_NONE;
}
