// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: presses the AW8637 step attenuator's + and - buttons.
//
// Board: any ESP32 (tested target: ESP-32S NodeMCU), Arduino core.
// Wiring (docs/BENCH.md): PIN_PLUS and PIN_MINUS each drive one relay
// channel (or photorelay, or NPN transistor) whose contacts are wired across
// that button: relay COM to one side of the button, NO to the other.
// GPIO32/33 leave GPIO25/26 (the DACs) free for a video pattern on the same
// ESP32.
//
// The attenuator has no readback, so the position is counted here:
//   1. One click adds 0.25 dB (the default "step 0.25"); the LEDs show the
//      value in binary (0.25, 0.5, 1, 2, 4, 8, 16 dB). If + and - together
//      selects a coarser step on the attenuator, set it and send the same
//      value with "step" for fewer clicks.
//   2. "home" presses - enough times to reach 0 dB from anywhere, or "at X"
//      declares the value the LEDs show now (no presses).
//   3. "set X" presses + or - to reach X dB (0 to 31.75, on the step grid).
//
// The owner's bench has ONE relay, on + (PIN_MINUS = -1). Then the sketch can
// only go up: set 0 dB by hand, "at 0", and "set" upwards. A lower value
// answers "ERR set down" and must be set by hand, followed by "at". "home"
// needs the - relay. With a second relay on GPIO33, set PIN_MINUS = 33.
//
// Serial 115200, one command per line, one "OK <cmd> ..." or "ERR <cmd> ..."
// reply each (same style as the C5 console):
//   step <0.25|0.5|1|10> | home | at <dB> | set <dB> | get |
//   press <+|-> [n] | timing <press_ms> <gap_ms>
// "get" also reports minus=yes|no, so a script can tell which setup it has.
//
// Test pattern (video.cpp, ntsc_frame.c): GPIO25 carries a 525-line
// interlaced monochrome NTSC picture for the VTX's video input, from boot
// (pattern card, 1 Vpp). DMA replays the frame, so presses never disturb it.
//   video <on|off> | pattern <card|bars|ramp|check|grey|black|white> |
//   amp <mV> (sync tip to white, 200..3200, default 1000) |
//   vget (state and the measured line rate, takes 1 s)
// Changing pattern or amp restarts the output: the receiver relocks.
//
// VTX control (tramp.cpp, IRC Tramp on the VTX's RX pad: P17 through
// 1 kohm, P16 direct; measure the pad first, docs/BENCH.md):
//   vtx (limits, channel, power, pit, temperature) | vtx mhz <MHz> |
//   vtx ch <band><1..8> (A B E F R, e.g. R8) | vtx mw <mW> [force] |
//   vtx pit <on|off> | vtx active <seconds>
// Every set is confirmed by reading the status back. Above 200 mW needs
// "force" (the VTX needs airflow there); the attenuator chain must come first.
// Pit guard: unless a bench run holds a lease ("vtx active S", up to 600 s,
// renewed by bench.py every step; "vtx active 0" ends it), the ESP32 checks
// the VTX every 5 s and puts it in pit mode, so it idles cool from boot, after
// a run, and if a run dies. "vtx pit off" without a lease is undone by the
// next check.

#include <Arduino.h>

#include <Preferences.h>

#include "ladder.h"
#include "net.h"
#include "tramp.h"
#include "video.h"

static const int PIN_PLUS = 32;
static const int PIN_MINUS = -1;  // -1: no relay on the - button; 33 with a second relay
// Low-level-trigger relay modules switch on when the input is LOW.
static const bool ACTIVE_LOW = true;
static const int QUARTERS_MAX = 127;  // 31.75 dB in 0.25 dB units

static int press_ms = 60;
static int gap_ms = 80;
static int unit = 1;        // attenuator step size in 0.25 dB units (1, 2, 4, 40)
static int position = -1;   // in 0.25 dB units, -1 = unknown until "home" or "at"

static ntsc_pattern_t vpattern = PAT_CARD;
static int vamp_mv = 1000;
static const int VAMP_MIN = 200, VAMP_MAX = 3200;  // white must stay under DAC code 255

static const int VTX_MW_SAFE = 200;  // above this only with "force"
static const uint32_t GUARD_PERIOD_MS = 5000;
static const long LEASE_MAX_S = 600;
static uint32_t lease_until_ms = 0;  // VTX may leave pit mode until then
static bool lease_on = false;
static uint32_t next_guard_ms = 0;   // 0: check at once (boot)

static bool lease_active()
{
  if (lease_on && (int32_t)(lease_until_ms - millis()) <= 0) lease_on = false;
  return lease_on;
}

// Puts the VTX in pit mode when no bench run holds a lease.
static void pit_guard()
{
  if ((int32_t)(millis() - next_guard_ms) < 0) return;
  next_guard_ms = millis() + GUARD_PERIOD_MS;
  if (lease_active()) return;
  TrampState st;
  const char *err = tramp_set_pit(true, &st);  // reads first, sends only if needed
  static bool reported = false;
  if (!err && !reported) {
    OUT.println("EVT vtx pit on (no bench run)");
    reported = true;
  }
  if (err) reported = false;  // VTX off or not answering: say so again when it is back
}

static void vtx_reply(const char *err, const TrampState &st)
{
  if (err) {
    OUT.printf("ERR vtx %s\n", err);
    return;
  }
  OUT.print("OK vtx");
  if (st.have_status)
    OUT.printf(" mhz=%u conf_mw=%u actual_mw=%u pit=%d lock=%d", st.mhz, st.conf_mw, st.actual_mw,
                  st.pit ? 1 : 0, st.race_lock ? 1 : 0);
  if (st.have_limits) OUT.printf(" min_mhz=%u max_mhz=%u max_mw=%u", st.min_mhz, st.max_mhz, st.max_mw);
  if (st.have_temp) OUT.printf(" temp=%d", st.temp);
  if (lease_active()) OUT.printf(" guard=active left_s=%ld", (long)((lease_until_ms - millis()) / 1000));
  else OUT.print(" guard=pit");
  OUT.println();
}

static void vtx_command(String arg)
{
  TrampState st;
  int sp = arg.indexOf(' ');
  String sub = sp < 0 ? arg : arg.substring(0, sp);
  String val = sp < 0 ? "" : arg.substring(sp + 1);
  val.trim();
  if (sub == "") {
    vtx_reply(tramp_query_all(&st) ? nullptr : "no answer from the VTX (wiring, power, Tramp?)", st);
  } else if (sub == "mhz" || sub == "ch") {
    long mhz = sub == "mhz" ? val.toInt() : (val.length() == 2 ? tramp_band_mhz(val[0], val[1] - '0') : 0);
    if (mhz < 5000 || mhz > 6100) {
      OUT.println("ERR vtx mhz 5000..6100, or ch like R8 (bands A B E F R)");
      return;
    }
    vtx_reply(tramp_set_mhz((uint16_t)mhz, &st), st);
  } else if (sub == "mw") {
    int sp2 = val.indexOf(' ');
    long mw = (sp2 < 0 ? val : val.substring(0, sp2)).toInt();
    bool force = sp2 > 0 && val.substring(sp2 + 1) == "force";
    if (mw < 1 || mw > 1000) {
      OUT.println("ERR vtx mw 1..1000");
      return;
    }
    if (mw > VTX_MW_SAFE && !force) {
      OUT.printf("ERR vtx mw above %d needs 'force' (airflow, attenuator chain first)\n", VTX_MW_SAFE);
      return;
    }
    vtx_reply(tramp_set_mw((uint16_t)mw, &st), st);
  } else if (sub == "pit" && (val == "on" || val == "off")) {
    vtx_reply(tramp_set_pit(val == "on", &st), st);
  } else if (sub == "active") {
    long s = val.toInt();
    if (s < 0 || s > LEASE_MAX_S || (s == 0 && val != "0")) {
      OUT.printf("ERR vtx active 0..%ld seconds\n", LEASE_MAX_S);
      return;
    }
    lease_on = s > 0;
    lease_until_ms = millis() + (uint32_t)s * 1000u;
    next_guard_ms = millis() + GUARD_PERIOD_MS;
    vtx_reply(tramp_set_pit(!lease_on, &st), st);
  } else {
    OUT.println("ERR vtx [mhz N | ch R8 | mw N [force] | pit on|off | active S]");
  }
}

// Video output: the 6-bit resistor ladder (colour, ladder.cpp) or the
// built-in DAC on GPIO25 (monochrome, video.cpp). Both use I2S0, so only one
// runs. Saved in flash; the ladder is the default since it was built.
static bool use_ladder = true;

// Nominal VTX deviation (the DSP's deviation_scale 1.0, measured
// 2026-11-04): the ladder's swing is set by its load, the DAC's by amp.
static int default_amp()
{
  return use_ladder ? 1600 : 1000;
}

static void load_output()
{
  Preferences p;
  p.begin("att_ctl", true);
  use_ladder = p.getBool("ladder", true);
  p.end();
  vamp_mv = default_amp();
}

static void save_output()
{
  Preferences p;
  p.begin("att_ctl", false);
  p.putBool("ladder", use_ladder);
  p.end();
}

static void video_off()
{
  ladder_stop();
  video_stop();
}

// Restarts the video with the current settings and replies for cmd.
static void video_reply(const char *cmd)
{
  video_off();
  const char *err = use_ladder ? ladder_start(vpattern, vamp_mv) : video_start(vpattern, vamp_mv);
  if (err) OUT.printf("ERR %s %s\n", cmd, err);
  else
    OUT.printf("OK %s video=on out=%s pattern=%s amp_mv=%d\n", cmd, use_ladder ? "ladder" : "dac",
               ntsc_pattern_name(vpattern), vamp_mv);
}

static void drive(int pin, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, (on != ACTIVE_LOW) ? HIGH : LOW);
}

static void press(int pin, int n) {
  if (pin < 0) return;
  for (int k = 0; k < n; k++) {
    drive(pin, true);
    delay(press_ms);
    drive(pin, false);
    delay(gap_ms);
  }
}

static int quarters(const String &db) {
  return (int)lroundf(db.toFloat() * 4.0f);
}

void setup() {
  // Output first, then level. Arduino-ESP32 3.x ignores digitalWrite() on a
  // pin that pinMode() has not attached yet, and pinMode(OUTPUT) starts LOW,
  // which holds a low-level relay on (button held, attenuator auto-repeats).
  // The LOW between the two calls lasts microseconds, far shorter than the
  // relay's pull-in time, so it does not click.
  pinMode(PIN_PLUS, OUTPUT);
  drive(PIN_PLUS, false);
  if (PIN_MINUS >= 0) {
    pinMode(PIN_MINUS, OUTPUT);
    drive(PIN_MINUS, false);
  }
  Serial.begin(115200);
  OUT.println(PIN_MINUS >= 0 ? "att_ctl ready (send 'step', then 'home' or 'at')"
                                : "att_ctl ready, + relay only (send 'step', set the LEDs by hand, then 'at')");
  load_output();
  video_reply("video");  // before Wi-Fi: the frame needs the big contiguous block
  tramp_begin();
  net_begin();
}

static void handle(String line) {
  line.trim();
  if (line.length() == 0) return;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? "" : line.substring(sp + 1);
  arg.trim();

  if (cmd == "step") {
    int u = quarters(arg);
    if (u != 1 && u != 2 && u != 4 && u != 40) {
      OUT.println("ERR step one of 0.25 0.5 1 10");
      return;
    }
    unit = u;
    OUT.printf("OK step db=%.2f\n", unit / 4.0f);
  } else if (cmd == "home") {
    if (PIN_MINUS < 0) {
      OUT.println("ERR home needs the - relay: set 0 dB by hand, then send at 0");
      return;
    }
    press(PIN_MINUS, (QUARTERS_MAX + unit - 1) / unit + 3);
    position = 0;
    OUT.println("OK home db=0");
  } else if (cmd == "at") {
    int q = quarters(arg);
    if (q < 0 || q > QUARTERS_MAX) {
      OUT.println("ERR at range 0..31.75");
      return;
    }
    position = q;
    OUT.printf("OK at db=%.2f\n", position / 4.0f);
  } else if (cmd == "set") {
    if (position < 0) {
      OUT.println("ERR set position unknown, send home or at");
      return;
    }
    int want = quarters(arg);
    if (want < 0 || want > QUARTERS_MAX) {
      OUT.println("ERR set range 0..31.75");
      return;
    }
    int diff = want - position;
    if (diff % unit != 0) {
      OUT.printf("ERR set %.2f dB is not on the %.2f dB step grid from %.2f\n",
                    want / 4.0f, unit / 4.0f, position / 4.0f);
      return;
    }
    if (diff < 0 && PIN_MINUS < 0) {
      OUT.printf("ERR set down: no - relay, set %.2f dB by hand, then send at\n", want / 4.0f);
      return;
    }
    if (diff > 0) press(PIN_PLUS, diff / unit);
    if (diff < 0) press(PIN_MINUS, -diff / unit);
    position = want;
    OUT.printf("OK set db=%.2f\n", position / 4.0f);
  } else if (cmd == "get") {
    const char *minus = PIN_MINUS >= 0 ? "yes" : "no";
    if (position < 0) OUT.printf("OK get db=unknown step=%.2f minus=%s\n", unit / 4.0f, minus);
    else OUT.printf("OK get db=%.2f step=%.2f minus=%s\n", position / 4.0f, unit / 4.0f, minus);
  } else if (cmd == "press") {
    int n = 1;
    int sp2 = arg.indexOf(' ');
    if (sp2 > 0) n = arg.substring(sp2 + 1).toInt();
    bool plus = arg.startsWith("+");
    if (!plus && PIN_MINUS < 0) {
      OUT.println("ERR press no - relay");
      return;
    }
    press(plus ? PIN_PLUS : PIN_MINUS, n);
    if (position >= 0) position = constrain(position + (plus ? n : -n) * unit, 0, QUARTERS_MAX);
    OUT.printf("OK press %s %d\n", plus ? "+" : "-", n);
  } else if (cmd == "timing") {
    int sp2 = arg.indexOf(' ');
    if (sp2 > 0) {
      press_ms = arg.substring(0, sp2).toInt();
      gap_ms = arg.substring(sp2 + 1).toInt();
    }
    OUT.printf("OK timing press_ms=%d gap_ms=%d\n", press_ms, gap_ms);
  } else if (cmd == "video") {
    if (arg == "on") {
      video_reply("video");
    } else if (arg == "off") {
      video_off();
      OUT.println("OK video video=off");
    } else {
      OUT.println("ERR video on or off");
    }
  } else if (cmd == "pattern") {
    int p = ntsc_pattern_from_name(arg.c_str());
    if (p < 0) {
      OUT.println("ERR pattern one of card bars ramp check grey black white");
      return;
    }
    vpattern = (ntsc_pattern_t)p;
    video_reply("pattern");
  } else if (cmd == "amp") {
    int mv = arg.toInt();
    if (mv < VAMP_MIN || mv > VAMP_MAX) {
      OUT.printf("ERR amp range %d..%d mV\n", VAMP_MIN, VAMP_MAX);
      return;
    }
    vamp_mv = mv;
    video_reply("amp");
  } else if (cmd == "vtx") {
    vtx_command(arg);
  } else if (cmd == "out") {
    if (arg != "ladder" && arg != "dac") {
      OUT.printf("ERR out ladder or dac (now %s)\n", use_ladder ? "ladder" : "dac");
      return;
    }
    use_ladder = arg == "ladder";
    save_output();
    vamp_mv = default_amp();
    video_reply("out");
  } else if (cmd == "vget") {
    if (use_ladder) {
      const bool on = ladder_running();
      const float fs = ladder_sample_hz();
      OUT.printf("OK vget video=%s out=ladder pattern=%s amp_mv=%d rate_hz=%.0f line_hz=%.1f\n", on ? "on" : "off",
                 ntsc_pattern_name(vpattern), vamp_mv, fs, fs / 910.0f);
    } else {
      bool on = video_running();
      float hz = video_measure_line_hz(1000);
      OUT.printf("OK vget video=%s out=dac pattern=%s amp_mv=%d rate_hz=%u line_hz=", on ? "on" : "off",
                 ntsc_pattern_name(vpattern), vamp_mv, NTSC_SAMPLE_HZ);
      if (hz > 0.0f) OUT.printf("%.1f\n", hz);
      else OUT.println(on ? "unmeasured" : "0");
    }
  } else if (net_command(cmd, arg)) {
    // wifissid, wifipass, wifi
  } else {
    OUT.printf("ERR %s unknown\n", cmd.c_str());
  }
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handle(buf);
      buf = "";
    } else if (buf.length() < 128) {
      buf += c;
    }
  }
  net_poll(handle);
  // Between commands only (a check blocks up to about half a second; the
  // UART and the TCP socket buffer what arrives meanwhile).
  if (buf.length() == 0 && !Serial.available() && !net_line_pending()) pit_guard();
}
