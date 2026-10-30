// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: console command parser (pure logic).
//
// Grammar and ranges follow docs/PROTOCOL.md section 1. The parser checks
// syntax and the ranges that do not depend on the hardware state; the
// executor checks the rest (gain table size, board fit, PHY support).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "openvrx_boards.h"
#include "openvrx_lanes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPENVRX_LINE_MAX 120u    // characters per command line, terminator excluded
#define OPENVRX_MAX_TOKENS 8u

#define OPENVRX_TUNE_MIN_MHZ 5180
#define OPENVRX_TUNE_MAX_MHZ 5945
#define OPENVRX_OFFSET_MAX_KHZ 2500
#define OPENVRX_DIV_MIN 6u
#define OPENVRX_DIV_MAX 1023u
#define OPENVRX_SLIP_MAX 64u
#define OPENVRX_GAIN_MAX_INDEX 255u
#define OPENVRX_FILTER_CODE_MAX 63
#define OPENVRX_FILTER_SKIRT_MAX 60
#define OPENVRX_DRIVE_MAX 3u
#define OPENVRX_PHASE_US_MIN 10u
#define OPENVRX_PHASE_US_MAX 10000000u
#define OPENVRX_PHASE_US_DEFAULT 1000u
#define OPENVRX_FILTER_CAL (-1)

typedef enum {
    OPENVRX_CMD_VER,
    OPENVRX_CMD_STATUS,
    OPENVRX_CMD_BOARD,
    OPENVRX_CMD_PINS,
    OPENVRX_CMD_LANES,
    OPENVRX_CMD_TUNE,
    OPENVRX_CMD_GAIN,
    OPENVRX_CMD_GAINS,
    OPENVRX_CMD_CLK,
    OPENVRX_CMD_SLIP,
    OPENVRX_CMD_P11,
    OPENVRX_CMD_FILTER,
    OPENVRX_CMD_BW,
    OPENVRX_CMD_DRIVE,
    OPENVRX_CMD_SELFTEST,
    OPENVRX_CMD_SAVE,
    OPENVRX_CMD_REBOOT,
    OPENVRX_CMD_DL,
    OPENVRX_CMD_PING,
    OPENVRX_CMD_COUNT,
} openvrx_cmd_kind_t;

typedef enum {
    OPENVRX_ERR_NONE = 0,
    OPENVRX_ERR_SYNTAX,
    OPENVRX_ERR_RANGE,
    OPENVRX_ERR_STATE,
    OPENVRX_ERR_UNSUPPORTED,
    OPENVRX_ERR_HW,
} openvrx_err_code_t;

typedef enum {
    OPENVRX_ST_OFF = 0,
    OPENVRX_ST_WALK,
    OPENVRX_ST_CONST,
    OPENVRX_ST_COUNT,
} openvrx_selftest_mode_t;

typedef enum {
    OPENVRX_CLK_DIV = 0,
    OPENVRX_CLK_ON,
    OPENVRX_CLK_OFF,
} openvrx_clk_action_t;

typedef struct {
    openvrx_cmd_kind_t kind;
    union {
        openvrx_board_id_t board;
        openvrx_preset_t preset;
        struct { int mhz; int off_khz; } tune;
        unsigned gain;
        struct { openvrx_clk_action_t action; unsigned div; } clk;
        unsigned slip_n;
        bool p11_on;
        struct { int code; int skirt; } filter;   // code OPENVRX_FILTER_CAL = calibrated
        unsigned bw_mhz;
        struct { unsigned lanes; int clk; } drive; // clk -1 = unchanged
        struct { openvrx_selftest_mode_t mode; uint32_t value; uint32_t phase_us; } selftest;
    } a;
} openvrx_cmd_t;

typedef struct {
    openvrx_err_code_t code;
    char text[64];   // short human text (may contain spaces)
} openvrx_err_t;

const char *openvrx_cmd_word(openvrx_cmd_kind_t kind);
const char *openvrx_err_code_name(openvrx_err_code_t code);

// Splits line in place into tokens (separated by spaces or tabs). Returns
// the token count, or -1 if there are more than OPENVRX_MAX_TOKENS.
int openvrx_tokenize(char *line, char *tok[OPENVRX_MAX_TOKENS]);

// Parses one command line (no line terminator). The line is modified.
// Returns true and fills cmd on success. On failure fills err; *word_out
// receives the command word to echo in the ERR reply (the first token, or
// "?" for an empty line).
bool openvrx_cmd_parse(char *line, openvrx_cmd_t *cmd, openvrx_err_t *err, const char **word_out);

// Strict number parsers: the whole string must be consumed.
bool openvrx_parse_int(const char *s, long min, long max, long *out);
bool openvrx_parse_hex(const char *s, uint32_t *out);

// Formats "ERR <word> <code> <text>" (no newline). Returns the length.
int openvrx_format_err(char *buf, size_t len, const char *word, const openvrx_err_t *err);

// Line assembler: feeds raw bytes from the serial port and yields complete
// lines. A '\r' immediately before '\n' is dropped; other '\r' are kept out
// of the line as well. Lines longer than OPENVRX_LINE_MAX are reported as
// overflow once their '\n' arrives.
typedef struct {
    char buf[OPENVRX_LINE_MAX + 1];
    size_t len;
    bool overflow;
    bool done;   // the previous feed completed a line
} openvrx_linebuf_t;

typedef enum {
    OPENVRX_LINE_NONE = 0,   // need more bytes
    OPENVRX_LINE_READY,      // buf holds a NUL-terminated line
    OPENVRX_LINE_OVERFLOW,   // the line was too long and has been discarded
} openvrx_line_status_t;

void openvrx_linebuf_init(openvrx_linebuf_t *lb);
// Feeds one byte. After READY/OVERFLOW the caller consumes the line and the
// next byte starts a new one.
openvrx_line_status_t openvrx_linebuf_feed(openvrx_linebuf_t *lb, char c);

#ifdef __cplusplus
}
#endif
