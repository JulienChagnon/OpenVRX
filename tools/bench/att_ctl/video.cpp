// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: NTSC test pattern output (see video.h).
#include "video.h"

#include <Arduino.h>
#include <stdlib.h>

#include "driver/dac_continuous.h"
#include "esp_heap_caps.h"

// The ESP32 DAC DMA takes 16-bit slots (SOC_DAC_DMA_16BIT_ALIGN), so one
// 8-bit sample costs 2 bytes. A DMA buffer holds 15 lines (3840 bytes, under
// the 4092 limit); 35 full buffers are one frame (134400 bytes).
// One descriptor more than that: dac_continuous_write_cyclically() (IDF 5.5)
// spreads the last two buffers' worth of data over two descriptors, half
// each, so with exactly 35 it drops the last 960 samples. With 36 the ring
// is 34 full buffers and two half ones, exactly one frame, so the loop
// restarts on frame line 1 and every frame raises DESC_NUM interrupts.
static const int LINES_PER_BUF = 15;
static const size_t BUF_BYTES = LINES_PER_BUF * NTSC_SPL * 2;
static const uint32_t DESC_NUM = NTSC_LINES / LINES_PER_BUF + 1;
static_assert(NTSC_LINES % LINES_PER_BUF == 0, "frame must fill whole DMA buffers");
static_assert(BUF_BYTES <= 4092, "DMA buffer too large");

// DAC step: full scale is about the 3.3 V rail over 256 codes.
static const float MV_PER_CODE = 3300.0f / 256.0f;

static dac_continuous_handle_t s_dac = nullptr;
static volatile uint32_t s_bufs = 0;

static bool IRAM_ATTR on_buf_done(dac_continuous_handle_t, const dac_event_data_t *, void *)
{
  s_bufs = s_bufs + 1;
  return false;
}

void video_stop()
{
  if (!s_dac) return;
  dac_continuous_disable(s_dac);
  dac_continuous_del_channels(s_dac);
  s_dac = nullptr;
}

bool video_running()
{
  return s_dac != nullptr;
}

const char *video_start(ntsc_pattern_t p, int amp_mv)
{
  video_stop();
  // The frame first: it needs the one big contiguous block (67 KB); the DMA
  // buffers are small and fit anywhere. Freed once the driver has copied it.
  uint8_t *frame = (uint8_t *)heap_caps_malloc(NTSC_FRAME_BYTES, MALLOC_CAP_8BIT);
  if (!frame) return "no memory for the frame";
  ntsc_build_frame(frame, p, ntsc_levels(amp_mv, MV_PER_CODE));
  // The ESP32's I2S sends the two 16-bit slots of each 32-bit FIFO word in
  // swapped order, so samples a b c d leave the DAC as b a d c. Seen on the
  // bench (Nov 4 2026) as a 1-sample sliver of the wrong level at every edge
  // that falls inside a pair. Swap each pair here so they come out in order.
  // Pairs never straddle a DMA buffer: every buffer holds an even count.
  for (size_t i = 0; i + 1 < NTSC_FRAME_BYTES; i += 2) {
    uint8_t t = frame[i];
    frame[i] = frame[i + 1];
    frame[i + 1] = t;
  }

  dac_continuous_config_t cfg = {
      .chan_mask = DAC_CHANNEL_MASK_CH0,  // GPIO25
      .desc_num = DESC_NUM,
      .buf_size = BUF_BYTES,
      .freq_hz = NTSC_SAMPLE_HZ,
      .offset = 0,
      .clk_src = DAC_DIGI_CLK_SRC_APLL,   // fine frequency steps: exact NTSC line rate
      .chan_mode = DAC_CHANNEL_MODE_SIMUL,
  };
  const char *err = nullptr;
  if (dac_continuous_new_channels(&cfg, &s_dac) != ESP_OK) {
    s_dac = nullptr;
    err = "dac channels (DMA memory?)";
  } else {
    dac_event_callbacks_t cb = {};
    cb.on_convert_done = on_buf_done;
    size_t loaded = 0;
    esp_err_t e;
    static char msg[80];
    if (dac_continuous_register_event_callback(s_dac, &cb, nullptr) != ESP_OK) {
      err = "dac callback";
    } else if (dac_continuous_enable(s_dac) != ESP_OK) {
      err = "dac enable";
    } else if ((e = dac_continuous_write_cyclically(s_dac, frame, NTSC_FRAME_BYTES, &loaded)) != ESP_OK ||
               loaded != NTSC_FRAME_BYTES) {
      snprintf(msg, sizeof msg, "dac cyclic write %s, loaded %u of %u", esp_err_to_name(e), (unsigned)loaded,
               (unsigned)NTSC_FRAME_BYTES);
      err = msg;
    }
    if (err) {
      dac_continuous_disable(s_dac);  // harmless if it never got enabled
      dac_continuous_del_channels(s_dac);
      s_dac = nullptr;
    }
  }
  free(frame);
  return err;
}

float video_measure_line_hz(int ms)
{
  if (!s_dac) return 0.0f;
  uint32_t b0 = s_bufs;
  uint32_t t0 = micros();
  delay(ms);
  uint32_t b1 = s_bufs;
  uint32_t t1 = micros();
  if (b1 == b0 || t1 == t0) return 0.0f;
  // DESC_NUM interrupts per frame of NTSC_LINES lines.
  return (float)(b1 - b0) * NTSC_LINES / DESC_NUM * 1e6f / (float)(t1 - t0);
}
