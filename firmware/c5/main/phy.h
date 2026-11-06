// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/phy.h at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d. Changes for OpenVRX: kept the
// recursive PHY ownership, the analog filter, the fixed digital RX filter
// and the 11p baseline; removed generations, actuator leases, observation
// hooks and C5VRX's DC DAC searches (the DC hold came back in a smaller
// form, phy_dco_*, for the per-row trims); added an 11p "off" path that returns to
// the stock flags and fields captured at boot, and filter writes that know
// whether 11p is on (the calibrated bytes differ).
//
// Writes outside the public PHY API run only on the libphy whose SHA-256
// the build verified (OPENVRX_PHY_PINNED); on any other PHY they refuse.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Recursive PHY transaction: every retune, filter or gain change holds it.
void phy_begin(void);
void phy_end(void);

// True when the pinned-PHY register paths are compiled in.
bool phy_pinned(void);

// Analog RX filter, after ESPARGOS esp-sdr (GPL-3.0) as used by C5VRX:
// BBTOP 0x67 regs 6/7 hold the RX0 capacitor code (low six bits), regs
// 8..13 the second stage (calibrated bytes plus a skirt offset). The factory
// bytes are captured once after PHY init. PHY_FILTER_CALIBRATED with 11p on
// writes the 11p baseline byte 60 (C5VRX); with 11p off it writes the
// factory bytes back.
#define PHY_FILTER_CALIBRATED (-1)
bool phy_filter_capture_base(void);
bool phy_filter_set_code(int code, bool p11);
bool phy_filter_set_skirt(int offset, bool p11);
int phy_filter_code(void);
int phy_filter_skirt(void);
// Mismatching registers between hardware and the bytes the shadow implies
// (0 = live filter as set), -1 when unsupported.
int phy_filter_hw_mismatch(bool p11);

// Fixed digital RX filter mode 12 (C5VRX bench result, PR #244/#279).
#define PHY_DFILT_MODE_DEFAULT 12
bool phy_dfilt_apply_default(void);
int phy_dfilt_hw_mode(void);

// 11p receive setting. phy_11p_capture_stock must run once after PHY init
// and before the first apply. apply_on: vendor phy_11p_set(1, 0) with
// verified entry and exact rollback (C5VRX PR #254). apply_off: stock
// flags and MMIO fields back (followed by a channel setup in rf.c).
void phy_11p_capture_stock(void);
esp_err_t phy_11p_apply_on(void);
esp_err_t phy_11p_apply_off(void);
// Flags/MMIO state: 1 = 11p fields set, 0 = stock fields, 2 = neither,
// -1 = unsupported.
int phy_11p_hw_state(void);
// Flags written before a channel setup so the vendor setup runs in the
// requested mode.
void phy_11p_prepare_setup(bool on);

// RX DC DACs ahead of the ADC (C5VRX's recovered libphy facts, same pinned
// archive): PBUS blocks 2 (I) and 3 (Q), bank 1 the vendor's coarse word
// per RF stage, bank 2 the fine pair of the gain row in force. In work mode
// every gain write replays the row's words from the AGC memory; a forced
// pair holds only in PBUS debug mode, where nothing is replayed. So a hold
// re-asserts the live gain and DC words (blocks 0, 1, 2, 3 and 8 only:
// copying all eleven blocks killed the IQ on C5VRX's board), and must be
// released before any gain write or channel setup (rf.c does). All refuse
// without the pinned PHY.
//
// Live words (works held or not): bank 1 and bank 2, I then Q.
bool phy_dco_read(int bank1[2], int bank2[2]);
// Enters debug mode with the live words, bank 1 forced to bank1 (NULL:
// as live, C5VRX's bank-1 pin), and bank 2 set to code. Verifies the read
// back; on a mismatch it releases and returns ESP_ERR_INVALID_RESPONSE.
// The live bank 2 before the hold (the vendor code of the row) goes to
// vendor_out, the live bank 1 to bank1_out (either may be NULL).
esp_err_t phy_dco_hold(const int bank1[2], const int code[2], int vendor_out[2], int bank1_out[2]);
// While held: a new bank-2 pair, verified.
esp_err_t phy_dco_set(const int code[2]);
// Back to work mode with the forced gain re-asserted (phy_pbus_workmode
// forces gain 50 for about 2 us). False when nothing was held.
bool phy_dco_release(void);
bool phy_dco_held(void);
