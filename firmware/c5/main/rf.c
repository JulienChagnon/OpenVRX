// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/rf.c at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d (C5VRX: "From
// legacy/c5vrx4/main/rf.c (C5VRX-2 wifi5.c lineage)"). Kept: the Wi-Fi
// bring-up as a receive-only front end, the continuous MODEM_DIAG streaming
// setup, the RX-only lock (MAC TX queues disabled), vendor AGC off with a
// forced gain index, off-grid tuning with phy_set_freq and the kHz offset,
// the bandwidth-before-setup rule and the restore_lock pattern (every
// tune/filter transaction restores the full receive state), and the
// download reboot. Changes for OpenVRX: tuning by raw MHz instead of band
// tables; 11p switchable (re-applied on every restore); lane routing moved
// to lanes_hw.c; DC hold, experimental patches, bank-1 and AGC memory
// checks removed; PHY readback state and a drift watchdog added; errors are
// reported instead of aborting.
#include "rf.h"

#include <stdlib.h>
#include <string.h>
#include "arc_gain.h"
#include "console.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "openvrx_cmd.h"
#include "phy.h"
#include "soc/lp_aon_reg.h"
#include "soc/soc.h"

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF 0x600a4d6cu
#define MAC_TXQ_STRIDE 0x10u
#define MAC_TXQ_ENABLE 0x80000000u
#define MAC_TXQ_COUNT 5u
// Continuous modem front end: keeps the ADC streaming into MODEM_DIAG
// without 802.11 packets (C5VRX-2 autonomous dump-first writer).
#define DUMP_CTRL 0x600a9004u
#define DUMP_PTR_MODE 0x600a9008u
#define DUMP_FORMAT 0x600a9018u
#define FE_PATH 0x600a20b4u
#define FE_ENABLE 0x600a0800u
#define SOURCE_CTRL 0x600a08ccu
#define SOURCE_MUX 0x600a70b8u
#define MODEM_CLOCK 0x600a9c04u
#define CTRL_ENABLE 0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK 0x01fe0000u
#define HP_SRAM_USAGE 0x60095004u
// Forced RX gain as the vendor PHY holds it: bit 23 force, bits 31:24 index
// (C5VRX phy.c pbus_workmode_keep_gain and pin_bank1 read it this way).
#define GAIN_FORCE_REG 0x600A702Cu
#define GAIN_FORCE_BIT (1u << 23)

extern int lmac_stop_hw_txq(void);
extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_set_freq(uint16_t freq_mhz, int offset);
extern void phy_chip_set_chan_offset(int offset_khz);
#ifdef OPENVRX_PHY_PINNED
extern unsigned char phy_param[];
#endif

static rf_config_t s_cfg;
static bool s_started;
static bool s_setup_valid, s_setup_bw20;
static uint16_t s_setup_mhz;
static arc_gain_table_t s_table;
static rf_phy_state_t s_state = RF_PHY_DOWN;
static bool s_gain_check;   // GAIN_FORCE_REG matched our model after the first restore

static esp_err_t lock_rx_only(void)
{
    (void)lmac_stop_hw_txq();
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q) REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q)
        if (REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

static bool rx_only_locked(void)
{
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q)
        if (REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) return false;
    return true;
}

static void enable_continuous_modem(void)
{
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;
    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;
    // Pre-trigger circular mode on TX_START: the MAC queues are disabled,
    // so it never fires and the writer streams continuously.
    REG32(DUMP_PTR_MODE) = (REG32(DUMP_PTR_MODE) & ~SELECTOR_MASK) | TX_START_SELECT;
    uint32_t ctrl = REG32(DUMP_CTRL);
    ctrl &= ~(CTRL_ENABLE | 0x00080000u | 0x00040000u);
    ctrl |= CTRL_DUMP_FIRST;
    ctrl = (ctrl & ~0x0001ffffu) | 16384u;
    REG32(DUMP_CTRL) = ctrl;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = ctrl | CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static bool gain_forced_as_set(void)
{
    const uint32_t v = REG32(GAIN_FORCE_REG);
    return (v & GAIN_FORCE_BIT) && ((v >> 24) & 0xffu) == s_cfg.gain;
}

// Every tuning/filter transaction restores the same receive state: modem
// streaming, RX-only lock, vendor AGC off, bandwidth, forced gain, 11p
// state, analog filter, digital filter. Caller holds phy_begin(). Returns
// true when everything read back as requested.
static bool restore_lock(void)
{
    bool ok = true;
    enable_continuous_modem();
    ok &= lock_rx_only() == ESP_OK;
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(s_cfg.bw20 ? 0u : 1u);
    phy_force_rx_gain(true, s_cfg.gain);
    if (!phy_pinned()) return ok;
    const esp_err_t p11 = s_cfg.p11 ? phy_11p_apply_on() : phy_11p_apply_off();
    if (p11 != ESP_OK) {
        console_log("11p %s failed: %s", s_cfg.p11 ? "on" : "off", esp_err_to_name(p11));
        ok = false;
    }
    // Written even at 0: leaving a skirt must undo it.
    if (!phy_filter_set_code(s_cfg.filter_code, s_cfg.p11)) ok = false;
    if (!phy_filter_set_skirt(s_cfg.skirt, s_cfg.p11)) ok = false;
    if (!phy_dfilt_apply_default()) ok = false;
    if (s_gain_check && !gain_forced_as_set()) ok = false;
    return ok;
}

static void set_state_after_restore(bool ok)
{
    s_state = !phy_pinned() ? (ok ? RF_PHY_UNSUPPORTED : RF_PHY_ERROR) : (ok ? RF_PHY_OK : RF_PHY_ERROR);
    if (!ok) console_evt("phy state=error");
}

static void capture_table(void)
{
#ifdef OPENVRX_PHY_PINNED
    if (phy_param[0x2a]) {
        arc_gain_table_from_bytes(&s_table, NULL, phy_param[0x126]);
    } else {
        uint8_t max = phy_param[0x125];
        if (max < 2u) max = phy_param[0x124];
        arc_gain_table_from_bytes(&s_table, &phy_param[0x422], max);
    }
#else
    // Unverified PHY: the generator's 5 GHz constants (84 entries).
    arc_gain_table_from_bytes(&s_table, NULL, ARC_5G_GAIN_MAX);
#endif
}

// Public 5 GHz centres. Off-grid frequencies are first tuned to the nearest
// one, then moved with phy_set_freq (how C5VRX reaches R8 and E8).
static const struct { uint8_t channel; uint16_t mhz; } s_centres[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720}, {149, 5745}, {153, 5765},
    {157, 5785}, {161, 5805}, {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

// Under phy_begin(). The bandwidth mode goes first: the setup builds the
// analog filter branch from it (C5VRX board 2026-10-19: in BW20 the RX0
// capacitor code sets the width, in BW40 the second-stage skirt does).
static esp_err_t channel_setup(uint16_t mhz)
{
    unsigned best = 0;
    for (unsigned k = 1; k < sizeof(s_centres) / sizeof(s_centres[0]); ++k)
        if (abs((int)mhz - (int)s_centres[k].mhz) < abs((int)mhz - (int)s_centres[best].mhz)) best = k;
    phy_11p_prepare_setup(s_cfg.p11);
    phy_wifi_fbw_sel(s_cfg.bw20 ? 0u : 1u);
    esp_err_t err = esp_wifi_set_channel(s_centres[best].channel, WIFI_SECOND_CHAN_NONE);
    if (err == ESP_OK && mhz != s_centres[best].mhz) phy_set_freq(mhz, 0);
    s_setup_valid = err == ESP_OK;
    if (s_setup_valid) {
        s_setup_bw20 = s_cfg.bw20;
        s_setup_mhz = mhz;
    }
    return err;
}

// Full retune at the configured frequency and offset. Caller holds phy_begin().
static esp_err_t retune(void)
{
    esp_err_t err = channel_setup(s_cfg.mhz);
    if (err == ESP_OK && s_cfg.off_khz) phy_chip_set_chan_offset(s_cfg.off_khz);
    const bool ok = restore_lock();
    if (err == ESP_OK) capture_table();   // a setup may regenerate the vendor gain table
    set_state_after_restore(ok && err == ESP_OK);
    return err != ESP_OK ? err : ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

