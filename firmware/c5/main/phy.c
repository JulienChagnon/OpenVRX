// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/phy.c at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d (itself from
// legacy/c5vrx4/main/phy_rx_lab.c, pinned C5 PHY investigation #150-153,
// #165). Changes for OpenVRX: generations, actuator leases, observation hooks,
// the digital filter selection menu and C5VRX's DC DAC searches removed (a
// smaller DC hold, phy_dco_*, from C5VRX main/phy.c, is at the end); filter
// writes take the 11p state into account; 11p off path added (stock flags
// and MMIO fields captured at boot). The 11p observation is credited by
// C5VRX to SushiDude (@Ready4Sushi).
#include "phy.h"

#include "arc_gain.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_lock_storage;
static SemaphoreHandle_t s_lock;

static void lock_take(void)
{
    portENTER_CRITICAL(&s_mux);
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_storage);
    portEXIT_CRITICAL(&s_mux);
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

static void lock_give(void)
{
    xSemaphoreGiveRecursive(s_lock);
}

void phy_begin(void) { lock_take(); }
void phy_end(void) { lock_give(); }

bool phy_pinned(void)
{
#ifdef OPENVRX_PHY_PINNED
    return true;
#else
    return false;
#endif
}

#ifdef OPENVRX_PHY_PINNED
extern uint8_t phy_i2c_readReg(uint8_t block, uint8_t host, uint8_t reg);
extern void phy_i2c_writeReg(uint8_t block, uint8_t host, uint8_t reg, uint8_t value);
extern void phy_rx_filter_mode(uint32_t mode);
extern void phy_11p_set(uint8_t enable, uint8_t mode);
extern unsigned char phy_param[];
static uint8_t s_filter_base[8];
static bool s_filter_base_valid;

#define FILTER_BLOCK 0x67u
#define FILTER_HOST 1u
#define FILTER_REG0 6u
#define P11_BASELINE_BYTE 60u
#endif
static int s_filter_code = PHY_FILTER_CALIBRATED, s_filter_skirt;

bool phy_filter_capture_base(void)
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    if (!s_filter_base_valid) {
        for (uint8_t i = 0; i < 8; ++i) s_filter_base[i] = phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i);
        s_filter_base_valid = true;
    }
    lock_give();
    return true;
#else
    return false;
#endif
}

#ifdef OPENVRX_PHY_PINNED
// The byte a filter register should hold for a code/skirt request.
static uint8_t want_byte(unsigned i, int code, int skirt, bool p11)
{
    if (i < 2u) {
        if (code == PHY_FILTER_CALIBRATED) return p11 ? P11_BASELINE_BYTE : s_filter_base[i];
        return (uint8_t)((s_filter_base[i] & ~63u) | (unsigned)code);
    }
    if (!skirt) return p11 ? P11_BASELINE_BYTE : s_filter_base[i];
    return arc_filter_code(s_filter_base[i], skirt);
}
#endif

// Verify every write; on mismatch return to the calibrated bytes.
bool phy_filter_set_code(int code, bool p11)
{
#ifdef OPENVRX_PHY_PINNED
    if (code != PHY_FILTER_CALIBRATED && (code < 0 || code > 63)) return false;
    lock_take();
    bool ok = s_filter_base_valid;
    for (uint8_t i = 0; ok && i < 2; ++i) {
        const uint8_t v = want_byte(i, code, 0, p11);
        phy_i2c_writeReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i, v);
        ok = phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i) == v;
    }
    if (ok) {
        s_filter_code = code;
    } else if (s_filter_base_valid) {
        for (uint8_t i = 0; i < 2; ++i)
            phy_i2c_writeReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i, want_byte(i, PHY_FILTER_CALIBRATED, 0, p11));
        s_filter_code = PHY_FILTER_CALIBRATED;
    }
    lock_give();
    return ok;
#else
    (void)code;
    (void)p11;
    return false;
#endif
}

bool phy_filter_set_skirt(int offset, bool p11)
{
#ifdef OPENVRX_PHY_PINNED
    if (offset < 0 || offset > 60) return false;
    lock_take();
    bool ok = s_filter_base_valid;
    for (uint8_t i = 2; ok && i < 8; ++i) {
        const uint8_t v = want_byte(i, 0, offset, p11);
        phy_i2c_writeReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i, v);
        ok = phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i) == v;
    }
    if (ok) {
        s_filter_skirt = offset;
    } else if (s_filter_base_valid) {
        for (uint8_t i = 2; i < 8; ++i)
            phy_i2c_writeReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i, want_byte(i, 0, 0, p11));
        s_filter_skirt = 0;
    }
    lock_give();
    return ok;
#else
    (void)offset;
    (void)p11;
    return false;
#endif
}

int phy_filter_code(void) { return s_filter_code; }
int phy_filter_skirt(void) { return s_filter_skirt; }

int phy_filter_hw_mismatch(bool p11)
{
#ifdef OPENVRX_PHY_PINNED
    if (!s_filter_base_valid) return -1;
    int bad = 0;
    lock_take();
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t w = want_byte(i, s_filter_code, s_filter_skirt, p11);
        bad += phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i) != w;
    }
    lock_give();
    return bad;
#else
    (void)p11;
    return -1;
#endif
}

// ---- Digital RX filter ---------------------------------------------------
// C5VRX bench #244/#279: mode 12 gave the better C/N with the VTX on. Reapply
// after every vendor setup (which changes the mode above 5830 MHz). Never
// touch the coupled ADC-rate selector.
#ifdef OPENVRX_PHY_PINNED
#define DFILT_REG 0x600A0430u
#define DFILT_SHIFT 18u
#define DFILT_MASK (UINT32_C(0xF) << DFILT_SHIFT)
#endif

int phy_dfilt_hw_mode(void)
{
#ifdef OPENVRX_PHY_PINNED
    return (int)((REG_READ(DFILT_REG) & DFILT_MASK) >> DFILT_SHIFT);
#else
    return -1;
#endif
}

bool phy_dfilt_apply_default(void)
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    phy_rx_filter_mode((uint32_t)PHY_DFILT_MODE_DEFAULT);
    const bool ok = phy_dfilt_hw_mode() == PHY_DFILT_MODE_DEFAULT;
    lock_give();
    return ok;
#else
    return false;
#endif
}

// ---- 11p receive setting -------------------------------------------------
// C5VRX PR #254: the vendor helper changes two phy_param flags, the eight
// analog filter bytes and four MMIO fields; (0, 0) is NOT a rollback, so the
// off path restores the stock values captured at boot instead.
#ifdef OPENVRX_PHY_PINNED
static const struct { uint32_t addr, mask, value; } s_11p_fields[] = {
    {0x600A7CE4u, 0x1cu, 0x10u},
    {0x600A7030u, 0x20u, 0},
    {0x600A7048u, 0x7f00u, 0x4000u},
    {0x600A71C4u, 0xfe0000u, 0x440000u},
};
#define P11_FIELDS (sizeof(s_11p_fields) / sizeof(s_11p_fields[0]))
static uint32_t s_stock_fields[P11_FIELDS];
static uint8_t s_stock_flags[2];
static bool s_stock_valid;
#endif

void phy_11p_capture_stock(void)
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    if (!s_stock_valid) {
        s_stock_flags[0] = phy_param[0x26];
        s_stock_flags[1] = phy_param[0x27];
        for (unsigned i = 0; i < P11_FIELDS; ++i)
            s_stock_fields[i] = REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask;
        s_stock_valid = true;
    }
    lock_give();
#endif
}

int phy_11p_hw_state(void)
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    bool on = phy_param[0x26] == 1 && phy_param[0x27] == 0;
    bool stock = s_stock_valid && phy_param[0x26] == s_stock_flags[0] && phy_param[0x27] == s_stock_flags[1];
    for (unsigned i = 0; i < P11_FIELDS; ++i) {
        const uint32_t v = REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask;
        on &= v == s_11p_fields[i].value;
        stock &= s_stock_valid && v == s_stock_fields[i];
    }
    lock_give();
    return on ? 1 : stock ? 0 : 2;
#else
    return -1;
#endif
}

void phy_11p_prepare_setup(bool on)
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    if (!on && s_stock_valid) {
        phy_param[0x26] = s_stock_flags[0];
        phy_param[0x27] = s_stock_flags[1];
    }
    lock_give();
#else
    (void)on;
#endif
}

esp_err_t phy_11p_apply_on(void)
{
#ifndef OPENVRX_PHY_PINNED
    return ESP_ERR_NOT_SUPPORTED;
#else
    lock_take();
    uint32_t saved[P11_FIELDS];
    uint8_t analog[8];
    const uint8_t flags[2] = {phy_param[0x26], phy_param[0x27]};
    for (unsigned i = 0; i < P11_FIELDS; ++i) saved[i] = REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask;
    for (uint8_t i = 0; i < 8; ++i) analog[i] = phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i);
    phy_11p_set(1, 0);
    bool ok = phy_11p_hw_state() == 1;
    for (uint8_t i = 0; i < 8; ++i) ok &= phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i) == P11_BASELINE_BYTE;
    esp_err_t result = ESP_OK;
    if (!ok) {
        phy_param[0x26] = flags[0];
        phy_param[0x27] = flags[1];
        for (uint8_t i = 0; i < 8; ++i) phy_i2c_writeReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i, analog[i]);
        for (unsigned i = 0; i < P11_FIELDS; ++i) {
            const uint32_t addr = s_11p_fields[i].addr, mask = s_11p_fields[i].mask;
            REG_WRITE(addr, (REG_READ(addr) & ~mask) | saved[i]);
        }
        bool restored = phy_param[0x26] == flags[0] && phy_param[0x27] == flags[1];
        for (uint8_t i = 0; i < 8; ++i)
            restored &= phy_i2c_readReg(FILTER_BLOCK, FILTER_HOST, FILTER_REG0 + i) == analog[i];
        for (unsigned i = 0; i < P11_FIELDS; ++i)
            restored &= (REG_READ(s_11p_fields[i].addr) & s_11p_fields[i].mask) == saved[i];
        result = restored ? ESP_ERR_INVALID_RESPONSE : ESP_FAIL;
    }
    lock_give();
    return result;
#endif
}

esp_err_t phy_11p_apply_off(void)
{
#ifndef OPENVRX_PHY_PINNED
    return ESP_ERR_NOT_SUPPORTED;
#else
    lock_take();
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (s_stock_valid) {
        phy_param[0x26] = s_stock_flags[0];
        phy_param[0x27] = s_stock_flags[1];
        for (unsigned i = 0; i < P11_FIELDS; ++i) {
            const uint32_t addr = s_11p_fields[i].addr, mask = s_11p_fields[i].mask;
            REG_WRITE(addr, (REG_READ(addr) & ~mask) | s_stock_fields[i]);
        }
        result = phy_11p_hw_state() == 0 ? ESP_OK : ESP_FAIL;
    }
    lock_give();
    return result;
#endif
}

// ---- RX DC DACs (C5VRX main/phy.c phy_dco_hold, pin_bank1 and
// pbus_workmode_keep_gain, same pinned libphy). Bank-2 pair of blocks 2/3
// in PBUS debug mode; see phy.h.
#ifdef OPENVRX_PHY_PINNED
extern uint16_t phy_pbus_rd(uint32_t block, uint32_t bank);
extern void phy_pbus_force_test(uint32_t block, uint32_t bank, uint32_t value);
extern void phy_pbus_debugmode(void);
extern void phy_pbus_workmode(void);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
#define PBUS_BLOCKS 9u
#define DCO_GAIN_FORCE_REG 0x600A702Cu
#define DCO_GAIN_FORCE_BIT (1u << 23)
// Only the gain/DC words the hold owns: BB (0), fine (1), DC (2, 3) and the
// 5 GHz RF code (8).
static inline bool pbus_owned(unsigned b) { return b <= 3u || b == 8u; }
static bool s_dco_held;

static void dco_read_live(uint16_t live[PBUS_BLOCKS][2])
{
    for (unsigned b = 0; b < PBUS_BLOCKS; ++b)
        if (pbus_owned(b))
            for (unsigned k = 0; k < 2; ++k) live[b][k] = phy_pbus_rd(b, k + 1);
}

static void dco_workmode_keep_gain(void)
{
    const uint32_t gain = REG_READ(DCO_GAIN_FORCE_REG);
    phy_pbus_workmode();
    if (gain & DCO_GAIN_FORCE_BIT) phy_force_rx_gain(true, (uint8_t)(gain >> 24));
}

static bool dco_pair_is(const int code[2])
{
    return phy_pbus_rd(2, 2) == (uint16_t)code[0] && phy_pbus_rd(3, 2) == (uint16_t)code[1];
}

static bool dco_code_ok(const int code[2])
{
    return code[0] >= 0 && code[0] <= 511 && code[1] >= 0 && code[1] <= 511;
}
#endif

bool phy_dco_read(int bank1[2], int bank2[2])
{
#ifdef OPENVRX_PHY_PINNED
    lock_take();
    bank1[0] = phy_pbus_rd(2, 1);
    bank1[1] = phy_pbus_rd(3, 1);
    bank2[0] = phy_pbus_rd(2, 2);
    bank2[1] = phy_pbus_rd(3, 2);
    lock_give();
    return true;
#else
    bank1[0] = bank1[1] = bank2[0] = bank2[1] = -1;
    return false;
#endif
}

esp_err_t phy_dco_hold(const int bank1[2], const int code[2], int vendor_out[2], int bank1_out[2])
{
#ifndef OPENVRX_PHY_PINNED
    (void)bank1;
    (void)code;
    (void)vendor_out;
    (void)bank1_out;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!dco_code_ok(code) || (bank1 && !dco_code_ok(bank1))) return ESP_ERR_INVALID_ARG;
    lock_take();
    if (s_dco_held) {
        lock_give();
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t live[PBUS_BLOCKS][2];
    dco_read_live(live);
    if (vendor_out) {
        vendor_out[0] = live[2][1];
        vendor_out[1] = live[3][1];
    }
    if (bank1_out) {
        bank1_out[0] = live[2][0];
        bank1_out[1] = live[3][0];
    }
    if (bank1) {
        live[2][0] = (uint16_t)bank1[0];
        live[3][0] = (uint16_t)bank1[1];
    }
    // Debug mode stops the work-mode replay: re-assert every owned word
    // first so gain and bank 1 stay as they were, then the pair.
    phy_pbus_debugmode();
    for (unsigned b = 0; b < PBUS_BLOCKS; ++b)
        if (pbus_owned(b))
            for (unsigned k = 0; k < 2; ++k) phy_pbus_force_test(b, k + 1, live[b][k]);
    phy_pbus_force_test(2, 2, (uint32_t)code[0]);
    phy_pbus_force_test(3, 2, (uint32_t)code[1]);
    s_dco_held = true;
    const bool ok = dco_pair_is(code) && phy_pbus_rd(2, 1) == live[2][0] && phy_pbus_rd(3, 1) == live[3][0];
    if (!ok) {
        dco_workmode_keep_gain();
        s_dco_held = false;
    }
    lock_give();
    return ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
#endif
}

esp_err_t phy_dco_set(const int code[2])
{
#ifndef OPENVRX_PHY_PINNED
    (void)code;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!dco_code_ok(code)) return ESP_ERR_INVALID_ARG;
    lock_take();
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_dco_held) {
        phy_pbus_force_test(2, 2, (uint32_t)code[0]);
        phy_pbus_force_test(3, 2, (uint32_t)code[1]);
        err = dco_pair_is(code) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
    }
    lock_give();
    return err;
#endif
}

bool phy_dco_release(void)
{
#ifndef OPENVRX_PHY_PINNED
    return false;
#else
    lock_take();
    const bool was = s_dco_held;
    if (was) {
        dco_workmode_keep_gain();
        s_dco_held = false;
    }
    lock_give();
    return was;
#endif
}

bool phy_dco_held(void)
{
#ifdef OPENVRX_PHY_PINNED
    return s_dco_held;
#else
    return false;
#endif
}
