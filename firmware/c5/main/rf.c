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
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "openvrx_cmd.h"
#include "openvrx_dc.h"
#include "openvrx_iq.h"
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
// One-shot capture into SRAM (rf_iq_capture, C5-20). HP_SRAM_USAGE is
// HP_SYSTEM_SRAM_USAGE_CONF (soc/hp_system_reg.h): bits 11:8 hand SRAM to
// the MAC dump, bit 16 adds 64 KB to its addresses. The vendor adctrig
// (librftest.a) captures with bits 11:8 = 2 and bit 16 set: the dump owns
// the bank from 0x40820000 and writes from 0x40830000 (the buffer its
// print_dump_data and loop_dump_test read). While it owns the bank nothing
// else may use it, DMA included (the sample clock's PARLIO buffer is heap),
// so the whole bank is kept out of the heap.
#define SRAM_USAGE_FIELD 0x00000f00u
#define SRAM_USAGE_DUMP 0x00000200u
#define SRAM_MAC_DUMP_ALLOC 0x00010000u
#define IQ_BANK_LO 0x40820000u
#define IQ_BANK_HI 0x40840000u
#define IQ_RAM 0x40830000u
#define IQ_RAM_WORDS 16384u
SOC_RESERVE_MEMORY_REGION(IQ_BANK_LO, IQ_BANK_HI, openvrx_iq_bank);
// DUMP_CTRL bits of a capture as adctrig sets them: count (16:0), done (18,
// read), software trigger pulse (19); 20 and 21 cleared; dump-first (17)
// off. DUMP_PTR_MODE bits 20:17 select the trigger: 15 is adctrig's
// software trigger (its mode 0). Never the TX selectors or adctrig's
// trigger cases 1 and 2: those transmit.
#define CTRL_TRIGGER 0x00080000u
#define CTRL_DONE 0x00040000u
#define CTRL_COUNT 0x0001ffffu
#define CTRL_CAPTURE_CLEAR 0x00300000u
#define TRIG_SELECT_MASK 0x001e0000u
#define TRIG_SELECT_SOFTWARE 0x001e0000u
#define DUMP_POINTER 0x0000ffffu
#define IQ_TIMEOUT_US 200u
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
static rf_tune_time_t s_tune_time;   // the last rf_tune (C5-22)

// DC hold (rf_dc_*). All under phy_begin().
static openvrx_dc_rows_t s_dc_rows;
static bool s_dc_failed;
static int s_dc_base[2] = {-1, -1}, s_dc_code[2] = {-1, -1}, s_dc_bank1[2] = {-1, -1};
// Bank 1 pinned per RF stage for one channel setup (C5VRX 2026-10-27: a
// vendor rewrite of bank 1 frozen into a hold moved every row of the stage
// beyond the trim range, until a reboot).
static int s_bank1_pin[ARC_RX_STAGE_COUNT][2];
static bool s_bank1_valid[ARC_RX_STAGE_COUNT];
static uint32_t s_bank1_overrides;
static volatile uint32_t s_dc_epoch;

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

// Back to PBUS work mode, so gain writes and channel setups replay the
// vendor rows. Caller holds phy_begin().
static void dc_release(void)
{
    if (phy_dco_release()) ++s_dc_epoch;
}

// Holds the DC pair of the gain row in force at its vendor code plus the
// row's trim. Caller holds phy_begin(), after the gain write.
static void dc_hold(void)
{
    if (!s_cfg.dc || s_dc_failed || !phy_pinned() || phy_dco_held()) return;
    arc_gain_tuple_t t;
    int bank1[2], base[2], trim[2];
    if (!arc_gain_tuple_decode(&s_table, s_cfg.gain, &t) || t.rf_stage >= ARC_RX_STAGE_COUNT ||
        s_cfg.gain >= OPENVRX_DC_ROWS || !phy_dco_read(bank1, base)) {   // work mode: the row's own words
        s_dc_failed = true;   // reported once; the watchdog leaves a failed hold alone
        console_evt("dc state=error cause=row gain=%u", s_cfg.gain);
        return;
    }
    if (!s_bank1_valid[t.rf_stage]) {
        s_bank1_pin[t.rf_stage][0] = bank1[0];
        s_bank1_pin[t.rf_stage][1] = bank1[1];
        s_bank1_valid[t.rf_stage] = true;
    } else if (bank1[0] != s_bank1_pin[t.rf_stage][0] || bank1[1] != s_bank1_pin[t.rf_stage][1]) {
        if (!s_bank1_overrides++)
            console_evt("dc bank1 moved stage=%u pin=%d/%d live=%d/%d", t.rf_stage, s_bank1_pin[t.rf_stage][0],
                        s_bank1_pin[t.rf_stage][1], bank1[0], bank1[1]);
    }
    openvrx_dc_rows_claim(&s_dc_rows, s_cfg.gain, trim);
    const int code[2] = {openvrx_dc_code(base[0], trim[0]), openvrx_dc_code(base[1], trim[1])};
    const esp_err_t err = phy_dco_hold(s_bank1_pin[t.rf_stage], code, NULL, NULL);
    ++s_dc_epoch;
    if (err != ESP_OK) {
        s_dc_failed = true;
        console_evt("dc state=error cause=hold err=%s", esp_err_to_name(err));
        return;
    }
    memcpy(s_dc_base, base, sizeof(s_dc_base));
    memcpy(s_dc_code, code, sizeof(s_dc_code));
    memcpy(s_dc_bank1, s_bank1_pin[t.rf_stage], sizeof(s_dc_bank1));
}

// A new channel setup: bank 1 legitimately differs per frequency group,
// so the pins start over; a failed hold gets another try.
static void dc_new_setup(void)
{
    memset(s_bank1_valid, 0, sizeof(s_bank1_valid));
    s_dc_failed = false;
}

// Every tuning/filter transaction restores the same receive state: modem
// streaming, RX-only lock, vendor AGC off, bandwidth, forced gain, 11p
// state, analog filter, digital filter, DC hold. Caller holds phy_begin().
// Returns true when everything read back as requested (a failed DC hold is
// reported on its own, rf_dc_info).
//
// The gain is forced again only after a channel setup (`setup`), or when
// the force no longer reads back as set (C5-22): re-writing a force that
// still holds disturbs the receiver (another C5 receiver measured 12 to
// 25 dB spikes after hops and a steady carrier 3 to 6 dB low; writing only
// on a change took its level sd from 2.2 to 0.1 dB). Without a working
// readback (s_gain_check false) it is always written.
static bool s_reforced;   // the last restore wrote the gain force

static bool restore_lock(bool setup)
{
    bool ok = true;
    dc_release();   // the restore must run in work mode (vendor rows replayed)
    enable_continuous_modem();
    ok &= lock_rx_only() == ESP_OK;
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(s_cfg.bw20 ? 0u : 1u);
    s_reforced = setup || !s_gain_check || !gain_forced_as_set();
    if (s_reforced) phy_force_rx_gain(true, s_cfg.gain);
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
    dc_hold();   // last: after the gain write, which replays the row's words
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
    dc_release();   // before the setup: in debug mode the vendor rows would not replay
    dc_new_setup();
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = channel_setup(s_cfg.mhz);
    if (err == ESP_OK && s_cfg.off_khz) phy_chip_set_chan_offset(s_cfg.off_khz);
    s_tune_time.vendor_us = (uint32_t)(esp_timer_get_time() - t0);
    const bool ok = restore_lock(true);
    s_tune_time.total_us = (uint32_t)(esp_timer_get_time() - t0);
    s_tune_time.setup = true;
    s_tune_time.reforced = s_reforced;
    if (err == ESP_OK) capture_table();   // a setup may regenerate the vendor gain table
    set_state_after_restore(ok && err == ESP_OK);
    return err != ESP_OK ? err : ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t rf_start(const rf_config_t *cfg)
{
    s_cfg = *cfg;
    // No esp_netif_init: no network interface exists.
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    // sta_disconnected_pm must be off: Wi-Fi power management would gate
    // the RF/PHY clocks that keep MODEM_DIAG streaming.
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    wcfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&wcfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK) return err;
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK) return err;
    wifi_bandwidths_t bandwidths = {.ghz_2g = WIFI_BW20, .ghz_5g = WIFI_BW40};
    if ((err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths)) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(173, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;
    // Promiscuous RX keeps the receive path alive; a zero filter keeps the
    // LMAC from buffering packets or raising interrupts.
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = {.filter_mask = 0};
    (void)esp_wifi_set_promiscuous_filter(&filter);
    if ((err = lock_rx_only()) != ESP_OK) return err;
    phy_begin();
    // The filter capacitors still hold the per-chip calibration from PHY
    // init and the 11p fields their stock values: capture both before
    // anything writes them.
    (void)phy_filter_capture_base();
    phy_11p_capture_stock();
    capture_table();
    if (s_cfg.gain > s_table.max_index) s_cfg.gain = s_table.max_index;
    phy_force_rx_gain(true, s_cfg.gain);
    s_gain_check = gain_forced_as_set();
    if (!s_gain_check) console_log("gain register readback not as modelled: gain watchdog off");
    s_started = true;
    err = retune();
    phy_end();
    return err == ESP_ERR_INVALID_RESPONSE ? ESP_OK : err;   // readback problems show in the state
}

bool rf_started(void) { return s_started; }

esp_err_t rf_tune(uint16_t mhz, int off_khz)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    phy_begin();
    esp_err_t err;
    if (s_setup_valid && mhz == s_setup_mhz && s_setup_bw20 == s_cfg.bw20) {
        // Offset only (as C5VRX's AFC): the vendor helper re-enables BB AGC,
        // so the full receive state is restored after it.
        s_cfg.off_khz = (int16_t)off_khz;
        const int64_t t0 = esp_timer_get_time();
        dc_release();
        dc_new_setup();
        phy_chip_set_chan_offset(off_khz);
        s_tune_time.vendor_us = (uint32_t)(esp_timer_get_time() - t0);
        const bool ok = restore_lock(false);
        s_tune_time.total_us = (uint32_t)(esp_timer_get_time() - t0);
        s_tune_time.setup = false;
        s_tune_time.reforced = s_reforced;
        set_state_after_restore(ok);
        err = ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
    } else {
        const rf_config_t old = s_cfg;
        s_cfg.mhz = mhz;
        s_cfg.off_khz = (int16_t)off_khz;
        err = retune();
        if (err != ESP_OK && err != ESP_ERR_INVALID_RESPONSE) {
            s_cfg = old;
            (void)retune();
        }
    }
    phy_end();
    return err;
}

esp_err_t rf_set_gain(uint8_t gain)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    if (gain > s_table.max_index) return ESP_ERR_INVALID_ARG;
    phy_begin();
    // Re-writing the gain in force while held changes nothing but would
    // leave debug mode for a moment (C5VRX rf_set_gain): skip it.
    if (gain == s_cfg.gain && phy_dco_held()) {
        phy_end();
        return ESP_OK;
    }
    dc_release();   // a hold keeps PBUS in debug mode, where the gain row would not replay
    phy_force_rx_gain(true, gain);
    s_cfg.gain = gain;
    ++s_dc_epoch;
    const bool ok = !s_gain_check || gain_forced_as_set();
    if (!ok) s_state = RF_PHY_ERROR;
    dc_hold();
    phy_end();
    return ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t rf_set_p11(bool on)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    if (!phy_pinned()) return ESP_ERR_NOT_SUPPORTED;
    phy_begin();
    s_cfg.p11 = on;
    const esp_err_t err = retune();   // the vendor setup must run in the new mode
    phy_end();
    return err;
}

esp_err_t rf_set_filter(int code, int skirt)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    if (!phy_pinned()) return (code == OPENVRX_FILTER_CAL && skirt == 0) ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    phy_begin();
    s_cfg.filter_code = (int8_t)code;
    s_cfg.skirt = (uint8_t)skirt;
    const bool ok = restore_lock(false);
    set_state_after_restore(ok);
    const bool applied = phy_filter_code() == code && phy_filter_skirt() == skirt;
    if (!applied) {
        // phy.c fell back to the calibrated bytes: report what is live.
        s_cfg.filter_code = (int8_t)phy_filter_code();
        s_cfg.skirt = (uint8_t)phy_filter_skirt();
    }
    phy_end();
    return ok && applied ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t rf_set_bw20(bool bw20)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    phy_begin();
    s_cfg.bw20 = bw20;
    // The analog filter branch follows the bandwidth at the last channel
    // setup, so a change repeats the setup (C5VRX rf_set_filter).
    const esp_err_t err = retune();
    phy_end();
    return err;
}

esp_err_t rf_dc_set_enabled(bool on)
{
    if (on && !phy_pinned()) return ESP_ERR_NOT_SUPPORTED;
    phy_begin();
    s_cfg.dc = on;
    s_dc_failed = false;
    esp_err_t err = ESP_OK;
    if (!on) {
        dc_release();
    } else if (s_started) {
        dc_hold();
        if (!phy_dco_held()) err = ESP_ERR_INVALID_RESPONSE;
    }
    phy_end();
    return err;
}

bool rf_dc_held(void) { return phy_dco_held(); }
uint32_t rf_dc_epoch(void) { return s_dc_epoch; }

esp_err_t rf_dc_step(uint8_t gain, const int step[2], int applied[2], bool limited[2])
{
    applied[0] = applied[1] = 0;
    limited[0] = limited[1] = false;
    if (gain >= OPENVRX_DC_ROWS) return ESP_ERR_INVALID_ARG;
    phy_begin();
    const bool live = phy_dco_held() && gain == s_cfg.gain;
    // A row not in force: its vendor code is not known here, so only the
    // code window around mid-scale bounds it (the real one applies at its
    // next hold).
    static const int mid[2] = {OPENVRX_DC_CODE_MAX / 2, OPENVRX_DC_CODE_MAX / 2};
    openvrx_dc_rows_step(&s_dc_rows, gain, live ? s_dc_base : mid, step, applied, limited);
    esp_err_t err = ESP_OK;
    if (live && (applied[0] || applied[1])) {
        const int code[2] = {openvrx_dc_code(s_dc_base[0], s_dc_rows.trim[gain][0]),
                             openvrx_dc_code(s_dc_base[1], s_dc_rows.trim[gain][1])};
        err = phy_dco_set(code);
        if (err == ESP_OK) {
            memcpy(s_dc_code, code, sizeof(s_dc_code));
        } else {
            dc_release();
            s_dc_failed = true;
            console_evt("dc state=error cause=set err=%s", esp_err_to_name(err));
        }
    }
    phy_end();
    return err;
}

void rf_dc_info(rf_dc_info_t *out)
{
    phy_begin();
    out->held = phy_dco_held();
    out->failed = s_dc_failed;
    for (unsigned a = 0; a < 2; ++a) {
        out->code[a] = out->held ? s_dc_code[a] : -1;
        out->base[a] = out->held ? s_dc_base[a] : -1;
        out->trim[a] = out->held ? s_dc_code[a] - s_dc_base[a] : 0;
        out->bank1[a] = out->held ? s_dc_bank1[a] : -1;
    }
    out->bank1_overrides = s_bank1_overrides;
    phy_end();
}

rf_config_t rf_config(void) { return s_cfg; }
rf_tune_time_t rf_last_tune(void) { return s_tune_time; }
rf_phy_state_t rf_phy_state(void) { return s_state; }

const char *rf_phy_state_name(rf_phy_state_t s)
{
    switch (s) {
    case RF_PHY_OK: return "ok";
    case RF_PHY_RESTORED: return "restored";
    case RF_PHY_ERROR: return "error";
    case RF_PHY_UNSUPPORTED: return "unpinned";
    default: return "down";
    }
}

uint8_t rf_gain_max(void) { return s_table.max_index; }

unsigned rf_gain_stage_starts(uint8_t *starts, unsigned max)
{
    uint8_t all[ARC_RX_STAGE_COUNT];
    const unsigned n = arc_gain_stage_starts(&s_table, all);
    const unsigned m = n < max ? n : max;
    memcpy(starts, all, m);
    return m;
}

void rf_watchdog_poll(void)
{
    if (!s_started || s_state == RF_PHY_ERROR) return;
    phy_begin();
    const char *why = NULL;
    if (!(REG32(DUMP_CTRL) & CTRL_ENABLE)) why = "modem";
    else if (!rx_only_locked()) why = "txq";
    else if (s_gain_check && !gain_forced_as_set()) why = "gain";
    else if (phy_pinned() && phy_11p_hw_state() != (s_cfg.p11 ? 1 : 0)) why = "p11";
    else if (s_cfg.dc && !s_dc_failed && phy_pinned()) {
        int bank1[2], bank2[2];
        if (!phy_dco_held()) why = "dco";
        else if (phy_dco_read(bank1, bank2) &&
                 (bank2[0] != s_dc_code[0] || bank2[1] != s_dc_code[1] || bank1[0] != s_dc_bank1[0] ||
                  bank1[1] != s_dc_bank1[1]))
            why = "dco";
    }
    if (why) {
        const bool ok = restore_lock(false);
        s_state = ok ? RF_PHY_RESTORED : RF_PHY_ERROR;
        console_evt("phy state=%s cause=%s", rf_phy_state_name(s_state), why);
    }
    phy_end();
}

// ---------------------------------------------------------------------
// One-shot 10-bit I/Q capture (C5-20)
// ---------------------------------------------------------------------

static portMUX_TYPE s_iq_mux = portMUX_INITIALIZER_UNLOCKED;

// The register part of a capture, from IRAM with interrupts off: while the
// dump engine owns the bank no code may touch it, and the lanes (fed by the
// same engine) pause from the first write to the last. Stops the stream,
// sets up the vendor's software-triggered capture of n words, pulses the
// trigger, waits for done (or the timeout), and restarts the stream from
// the exact register values it found, the way enable_continuous_modem
// starts it (enable last).
static IRAM_ATTR bool iq_capture_regs(unsigned n, uint32_t timeout_cycles, uint32_t *cycles, uint32_t *ptr)
{
    const uint32_t ctrl = REG32(DUMP_CTRL), mode = REG32(DUMP_PTR_MODE), sram = REG32(HP_SRAM_USAGE);
    const uint32_t cap = (ctrl & ~(CTRL_ENABLE | CTRL_DUMP_FIRST | CTRL_TRIGGER | CTRL_DONE | CTRL_CAPTURE_CLEAR |
                                   CTRL_COUNT)) | (n & CTRL_COUNT);
    const uint32_t t0 = esp_cpu_get_cycle_count();
    REG32(DUMP_CTRL) = ctrl & ~CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_PTR_MODE) = (mode & ~TRIG_SELECT_MASK) | TRIG_SELECT_SOFTWARE;
    REG32(HP_SRAM_USAGE) = (sram & ~SRAM_USAGE_FIELD) | SRAM_USAGE_DUMP | SRAM_MAC_DUMP_ALLOC;
    REG32(DUMP_CTRL) = cap;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = cap | CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = cap | CTRL_ENABLE | CTRL_TRIGGER;
    REG32(DUMP_CTRL) = cap | CTRL_ENABLE;
    bool done;
    while (!(done = (REG32(DUMP_CTRL) & CTRL_DONE) != 0) && esp_cpu_get_cycle_count() - t0 < timeout_cycles) {
    }
    *ptr = REG32(DUMP_PTR_MODE) & DUMP_POINTER;
    REG32(DUMP_CTRL) = cap;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(HP_SRAM_USAGE) = sram;
    REG32(DUMP_PTR_MODE) = mode;
    REG32(DUMP_CTRL) = ctrl & ~(CTRL_ENABLE | CTRL_TRIGGER | CTRL_DONE);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = (ctrl & ~(CTRL_TRIGGER | CTRL_DONE)) | CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    *cycles = esp_cpu_get_cycle_count() - t0;
    return done;
}

// Fills words [lo, hi) of the capture RAM with their fill pattern (the CPU owns the bank here).
static void iq_fill(unsigned lo, unsigned hi)
{
    volatile uint32_t *const ram = (volatile uint32_t *)(uintptr_t)IQ_RAM;
    for (unsigned k = lo; k < hi; ++k) ram[k] = openvrx_iq_sentinel(k);
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

// Caller holds phy_begin(). n words plus one guard word are filled; with
// `whole`, the full 16384 (for rf_iq_probe: where did the words land).
static esp_err_t iq_capture_locked(unsigned n, bool whole, rf_iq_meta_t *m)
{
    memset(m, 0, sizeof(*m));
    if (!s_started || s_state == RF_PHY_ERROR || s_state == RF_PHY_DOWN) return ESP_ERR_INVALID_STATE;
    if (!(REG32(DUMP_CTRL) & CTRL_ENABLE)) return ESP_ERR_INVALID_STATE;   // the watchdog restores the stream first
    iq_fill(0, whole ? IQ_RAM_WORDS : n + 1u);
    const uint32_t timeout = IQ_TIMEOUT_US * esp_rom_get_cpu_ticks_per_us();
    uint32_t cycles, ptr;
    portENTER_CRITICAL(&s_iq_mux);
    const bool done = iq_capture_regs(n, timeout, &cycles, &ptr);
    portEXIT_CRITICAL(&s_iq_mux);
    m->us = cycles / esp_rom_get_cpu_ticks_per_us();
    m->ptr = ptr;
    m->done = done;
    volatile const uint32_t *const ram = (volatile const uint32_t *)(uintptr_t)IQ_RAM;
    for (unsigned k = 0; k < n; ++k) m->written += ram[k] != openvrx_iq_sentinel(k);
    m->guard_ok = ram[n] == openvrx_iq_sentinel(n);
    if (!done) return ESP_ERR_TIMEOUT;
    return m->written == n && m->guard_ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t rf_iq_capture(uint32_t *words, unsigned n, rf_iq_meta_t *meta)
{
    if (n == 0 || n > OPENVRX_IQ_MAX) return ESP_ERR_INVALID_ARG;
    phy_begin();
    const esp_err_t err = iq_capture_locked(n, false, meta);
    volatile const uint32_t *const ram = (volatile const uint32_t *)(uintptr_t)IQ_RAM;
    for (unsigned k = 0; k < n; ++k) words[k] = ram[k];
    phy_end();
    return err;
}

esp_err_t rf_iq_probe(uint32_t *words, unsigned n, rf_iq_probe_t *p)
{
    memset(p, 0, sizeof(*p));
    if (n == 0 || n > OPENVRX_IQ_MAX) return ESP_ERR_INVALID_ARG;
    phy_begin();
    const esp_err_t err = iq_capture_locked(n, true, &p->meta);
    volatile const uint32_t *const ram = (volatile const uint32_t *)(uintptr_t)IQ_RAM;
    p->first = -1;
    p->last = -1;
    for (unsigned k = 0; k < IQ_RAM_WORDS; ++k) {
        if (ram[k] == openvrx_iq_sentinel(k)) continue;
        ++p->changed;
        if (p->first < 0) p->first = (int)k;
        p->last = (int)k;
    }
    for (unsigned k = 0; k < n; ++k) words[k] = ram[k];
    p->gain_forced = s_cfg.gain;
    phy_end();
    return err;
}

// Resets do not reach the modem domain; the forced clocks and the running
// dump writer would keep the ROM loader from answering and stall the next
// app boot. Stop both before any restart.
static void quiesce_modem(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    REG32(DUMP_CTRL) &= ~CTRL_ENABLE;
    REG32(HP_SRAM_USAGE) &= ~0x00010f00u;
    if (s_started) (void)esp_wifi_stop();
}

void rf_reboot(void)
{
    quiesce_modem();
    esp_restart();
}

void rf_reboot_to_download(void)
{
    quiesce_modem();
    REG_SET_FIELD(LP_AON_SYS_CFG_REG, LP_AON_FORCE_DOWNLOAD_BOOT, 1u);
    esp_restart();
}
