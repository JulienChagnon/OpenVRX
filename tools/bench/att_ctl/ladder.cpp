// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: colour NTSC on the resistor ladder (see ladder.h).
#include "ladder.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "clk_ctrl_os.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "esp_rom_lldesc.h"
#include "esp_rom_sys.h"
#include "hal/i2s_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_struct.h"

static const int PINS[6] = {18, 19, 21, 22, 23, 26};  // bit 0 (LSB) .. bit 5

// 4 x fsc timing (one sample 69.84 ns), NTSC values as in ntsc_frame.h.
static const int SPL = 910, HALF = 455;
static const int HSYNC = 67, EQ = 33, BROAD = 388;  // 4.68, 2.30, 27.10 us
static const int BURST0 = 76, BURSTN = 36;          // 5.31 us, 9 cycles
static const int ACT0 = 135, ACTW = 754;            // 9.43 us, 52.66 us
static const int LOOP_LINES = 2 * NTSC_LINES;       // colour frame pair
static const int BUF_BYTES = SPL * 2;               // 16-bit slot per sample

// APLL / mclk 2 / bck 2 / 2 (LCD mode sends each slot twice) = 4 fsc.
static const uint32_t APLL_HZ = 114545454u;
static const uint32_t DIV_TOTAL = 8;

// Ladder DAC step, open circuit: 63 codes = about 3.3 V * 122/(122+200) ohm.
static const float MV_PER_CODE = 2050.0f / 63.0f;
static const float BASE_CODE = 2.0f;  // sync tip, off the bottom rail

enum Band : int8_t {
  B_NONE = -1, B_BARS, B_MULTI, B_RAMP, B_GREY, B_BLACK, B_WHITE, B_CHECK0, B_CHECK1
};

struct Key {
  int8_t p0, p1;   // ntsc_pulse_t at 0H and H/2
  int8_t phase;    // subcarrier phase (0 or 2 samples) of burst/colour lines, -1 if none
  int8_t band;
};

static const int MAX_BUFS = 40;
static Key s_keys[MAX_BUFS];
static uint16_t *s_bufs[MAX_BUFS];
static int s_nbufs = 0;
static lldesc_t *s_desc = nullptr;
static bool s_running = false;
static bool s_apll = false;
static uint32_t s_real_apll = 0;

static float s_per_ire = 0.22f;

static uint16_t code(float ire)
{
  float c = BASE_CODE + (ire + 40.0f) * s_per_ire;
  if (c < 0.0f) c = 0.0f;
  if (c > 63.0f) c = 63.0f;
  uint16_t v = (uint16_t)lroundf(c);
  return (uint16_t)((v << 8) | v);  // both bytes: whichever lane the pins take
}

static int pulse_len(int p)
{
  switch (p) {
  case NTSC_PULSE_HSYNC: return HSYNC;
  case NTSC_PULSE_EQ: return EQ;
  case NTSC_PULSE_BROAD: return BROAD;
  default: return 0;
  }
}

static Band band_of(ntsc_pattern_t p, int row)
{
  switch (p) {
  case PAT_CARD:
    if (row < 120) return B_BARS;
    if (row < 160) return B_MULTI;
    if (row < 200) return B_RAMP;
    return B_GREY;
  case PAT_BARS: return B_BARS;
  case PAT_RAMP: return B_RAMP;
  case PAT_CHECK: return ((row / 20) & 1) ? B_CHECK1 : B_CHECK0;
  case PAT_GREY: return B_GREY;
  case PAT_BLACK: return B_BLACK;
  case PAT_WHITE:
  default: return B_WHITE;
  }
}

// 75 % colour bars, R'G'B' (gamma coded): white, yellow, cyan, green,
// magenta, red, blue.
static const float BARS[7][3] = {
    {0.75f, 0.75f, 0.75f}, {0.75f, 0.75f, 0.0f}, {0.0f, 0.75f, 0.75f}, {0.0f, 0.75f, 0.0f},
    {0.75f, 0.0f, 0.75f},  {0.75f, 0.0f, 0.0f},  {0.0f, 0.0f, 0.75f},
};
// Multiburst frequencies (MHz), five zones.
static const float MULTI_MHZ[5] = {0.5f, 1.0f, 2.0f, 3.0f, 4.0f};

// Subcarrier at sample phase p (0..3): sin and cos of p * 90 degrees.
static const float SIN4[4] = {0.0f, 1.0f, 0.0f, -1.0f};
static const float COS4[4] = {1.0f, 0.0f, -1.0f, 0.0f};

// IRE of picture sample x (0..ACTW-1) at absolute line sample n.
static float picture_ire(Band b, int x, int n, int phase)
{
  float r, g, bl;
  float y;
  switch (b) {
  case B_BARS: {
    const float *c = BARS[x * 7 / ACTW];
    r = c[0];
    g = c[1];
    bl = c[2];
    y = 0.299f * r + 0.587f * g + 0.114f * bl;
    const float u = 0.492111f * (bl - y), v = 0.877283f * (r - y);
    const int p = (n + phase) & 3;
    return 7.5f + 92.5f * y + 92.5f * (u * SIN4[p] + v * COS4[p]);
  }
  case B_MULTI: {
    const int zone = x * 5 / ACTW;
    const float cyc = MULTI_MHZ[zone] * 1e6f / (4.0f * 3579545.45f);
    y = 0.5f + 0.4f * sinf(2.0f * (float)M_PI * cyc * (float)x);
    break;
  }
  case B_RAMP: y = (float)x / (float)(ACTW - 1); break;
  case B_GREY: y = 0.5f; break;
  case B_BLACK: y = 0.0f; break;
  case B_CHECK0: y = ((x / 57) & 1) ? 1.0f : 0.0f; break;
  case B_CHECK1: y = ((x / 57) & 1) ? 0.0f : 1.0f; break;
  case B_WHITE:
  default: y = 1.0f; break;
  }
  return 7.5f + 92.5f * y;
}

static void render(const Key &k, uint16_t *buf)
{
  static float ire[SPL];
  for (int n = 0; n < SPL; n++) ire[n] = 0.0f;
  for (int n = 0; n < pulse_len(k.p0); n++) ire[n] = -40.0f;
  for (int n = 0; n < pulse_len(k.p1); n++) ire[HALF + n] = -40.0f;
  if (k.phase >= 0) {
    // Burst on the -U axis: -20 IRE * sin.
    for (int n = BURST0; n < BURST0 + BURSTN; n++) ire[n] = -20.0f * SIN4[(n + k.phase) & 3];
  }
  if (k.band >= 0)
    for (int x = 0; x < ACTW; x++) ire[ACT0 + x] = picture_ire((Band)k.band, x, ACT0 + x, k.phase);
  // Samples leave in swapped pairs (the I2S sends the upper 16-bit slot of
  // each 32-bit word first), as on the built-in DAC: store at n ^ 1.
  for (int n = 0; n < SPL; n++) buf[n ^ 1] = code(ire[n]);
}

static int buffer_for(const Key &k)
{
  for (int i = 0; i < s_nbufs; i++)
    if (memcmp(&s_keys[i], &k, sizeof k) == 0) return i;
  if (s_nbufs >= MAX_BUFS) return -1;
  uint16_t *b = (uint16_t *)heap_caps_malloc(BUF_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!b) return -1;
  render(k, b);
  s_keys[s_nbufs] = k;
  s_bufs[s_nbufs] = b;
  return s_nbufs++;
}

static void free_all()
{
  for (int i = 0; i < s_nbufs; i++) heap_caps_free(s_bufs[i]);
  s_nbufs = 0;
  if (s_desc) heap_caps_free(s_desc);
  s_desc = nullptr;
}

void ladder_stop()
{
  if (s_running) {
    i2s_dev_t *hw = &I2S0;
    i2s_ll_tx_stop(hw);
    i2s_ll_tx_stop_link(hw);
    for (int pin : PINS) {
      gpio_reset_pin((gpio_num_t)pin);
      gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    }
    s_running = false;
  }
  if (s_apll) {
    periph_rtc_apll_release();
    s_apll = false;
  }
  free_all();
}

bool ladder_running()
{
  return s_running;
}

float ladder_sample_hz()
{
  return s_running ? (float)s_real_apll / (float)DIV_TOTAL : 0.0f;
}

const char *ladder_start(ntsc_pattern_t p, int amp_mv)
{
  ladder_stop();
  s_per_ire = (float)amp_mv / 140.0f / MV_PER_CODE;

  s_desc = (lldesc_t *)heap_caps_calloc(LOOP_LINES, sizeof(lldesc_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!s_desc) return "no memory for descriptors";
  for (int L = 0; L < LOOP_LINES; L++) {
    const int line = L % NTSC_LINES + 1;
    // 910 samples per line = 227.5 subcarrier cycles: phase 0 or 180 deg.
    const int8_t phase = (L & 1) ? 2 : 0;
    ntsc_pulse_t pl[2];
    ntsc_line_pulses(line, pl);
    const int row = ntsc_row(line);
    Key k;
    k.p0 = (int8_t)pl[0];
    k.p1 = (int8_t)pl[1];
    k.phase = pl[0] == NTSC_PULSE_HSYNC ? phase : -1;  // no burst in the vertical sync
    k.band = row >= 0 ? (int8_t)band_of(p, row) : (int8_t)B_NONE;
    const int b = buffer_for(k);
    if (b < 0) {
      free_all();
      return "no memory for line buffers";
    }
    lldesc_t *d = &s_desc[L];
    d->size = BUF_BYTES;
    d->length = BUF_BYTES;
    d->owner = 1;
    d->eof = 0;
    d->buf = (const uint8_t *)s_bufs[b];
    d->qe.stqe_next = &s_desc[(L + 1) % LOOP_LINES];
  }

  periph_rtc_apll_acquire();
  s_apll = true;
  if (periph_rtc_apll_freq_set(APLL_HZ, &s_real_apll) != ESP_OK) {
    ladder_stop();
    return "APLL refused 114.545 MHz";
  }

  periph_module_reset(PERIPH_I2S0_MODULE);
  periph_module_enable(PERIPH_I2S0_MODULE);
  i2s_dev_t *hw = &I2S0;  // clock gate: periph_module_enable above
  i2s_ll_tx_stop(hw);
  i2s_ll_tx_reset(hw);
  i2s_ll_tx_reset_dma(hw);
  i2s_ll_tx_reset_fifo(hw);
  i2s_ll_tx_clk_set_src(hw, I2S_CLK_SRC_APLL);
  i2s_ll_set_raw_mclk_div(hw, 2, 1, 0);
  i2s_ll_tx_set_bck_div_num(hw, 2);
  i2s_ll_enable_lcd(hw, true);
  i2s_ll_tx_stop_on_fifo_empty(hw, false);
  i2s_ll_tx_set_slave_mod(hw, false);
  i2s_ll_tx_set_bits_mod(hw, 16);
  i2s_ll_tx_select_std_slot(hw, I2S_STD_SLOT_BOTH, true);
  i2s_ll_tx_enable_right_first(hw, true);
  i2s_ll_dma_enable_owner_check(hw, false);       // a ring: never hand descriptors back
  i2s_ll_dma_enable_auto_write_back(hw, false);
  i2s_ll_dma_enable_eof_on_fifo_empty(hw, false);
  i2s_ll_set_out_link_addr(hw, (uint32_t)&s_desc[0]);
  i2s_ll_enable_dma(hw, true);

  for (int bit = 0; bit < 6; bit++) {
    const int pin = PINS[bit];
    esp_rom_gpio_pad_select_gpio(pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin, I2S0O_DATA_OUT8_IDX + bit, false, false);
  }

  i2s_ll_start_out_link(hw);
  esp_rom_delay_us(1);
  i2s_ll_tx_start(hw);
  s_running = true;
  return nullptr;
}
