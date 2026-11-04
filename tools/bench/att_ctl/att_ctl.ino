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

#include <Arduino.h>

static const int PIN_PLUS = 32;
static const int PIN_MINUS = -1;  // -1: no relay on the - button; 33 with a second relay
// Low-level-trigger relay modules switch on when the input is LOW.
static const bool ACTIVE_LOW = true;
static const int QUARTERS_MAX = 127;  // 31.75 dB in 0.25 dB units

static int press_ms = 60;
static int gap_ms = 80;
static int unit = 1;        // attenuator step size in 0.25 dB units (1, 2, 4, 40)
static int position = -1;   // in 0.25 dB units, -1 = unknown until "home" or "at"

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
  // Level first, then output: the relay must not click on at boot.
  drive(PIN_PLUS, false);
  pinMode(PIN_PLUS, OUTPUT);
  if (PIN_MINUS >= 0) {
    drive(PIN_MINUS, false);
    pinMode(PIN_MINUS, OUTPUT);
  }
  Serial.begin(115200);
  Serial.println(PIN_MINUS >= 0 ? "att_ctl ready (send 'step', then 'home' or 'at')"
                                : "att_ctl ready, + relay only (send 'step', set the LEDs by hand, then 'at')");
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
      Serial.println("ERR step one of 0.25 0.5 1 10");
      return;
    }
    unit = u;
    Serial.printf("OK step db=%.2f\n", unit / 4.0f);
  } else if (cmd == "home") {
    if (PIN_MINUS < 0) {
      Serial.println("ERR home needs the - relay: set 0 dB by hand, then send at 0");
      return;
    }
    press(PIN_MINUS, (QUARTERS_MAX + unit - 1) / unit + 3);
    position = 0;
    Serial.println("OK home db=0");
  } else if (cmd == "at") {
    int q = quarters(arg);
    if (q < 0 || q > QUARTERS_MAX) {
      Serial.println("ERR at range 0..31.75");
      return;
    }
    position = q;
    Serial.printf("OK at db=%.2f\n", position / 4.0f);
  } else if (cmd == "set") {
    if (position < 0) {
      Serial.println("ERR set position unknown, send home or at");
      return;
    }
    int want = quarters(arg);
    if (want < 0 || want > QUARTERS_MAX) {
      Serial.println("ERR set range 0..31.75");
      return;
    }
    int diff = want - position;
    if (diff % unit != 0) {
      Serial.printf("ERR set %.2f dB is not on the %.2f dB step grid from %.2f\n",
                    want / 4.0f, unit / 4.0f, position / 4.0f);
      return;
    }
    if (diff < 0 && PIN_MINUS < 0) {
      Serial.printf("ERR set down: no - relay, set %.2f dB by hand, then send at\n", want / 4.0f);
      return;
    }
    if (diff > 0) press(PIN_PLUS, diff / unit);
    if (diff < 0) press(PIN_MINUS, -diff / unit);
    position = want;
    Serial.printf("OK set db=%.2f\n", position / 4.0f);
  } else if (cmd == "get") {
    const char *minus = PIN_MINUS >= 0 ? "yes" : "no";
    if (position < 0) Serial.printf("OK get db=unknown step=%.2f minus=%s\n", unit / 4.0f, minus);
    else Serial.printf("OK get db=%.2f step=%.2f minus=%s\n", position / 4.0f, unit / 4.0f, minus);
  } else if (cmd == "press") {
    int n = 1;
    int sp2 = arg.indexOf(' ');
    if (sp2 > 0) n = arg.substring(sp2 + 1).toInt();
    bool plus = arg.startsWith("+");
    if (!plus && PIN_MINUS < 0) {
      Serial.println("ERR press no - relay");
      return;
    }
    press(plus ? PIN_PLUS : PIN_MINUS, n);
    if (position >= 0) position = constrain(position + (plus ? n : -n) * unit, 0, QUARTERS_MAX);
    Serial.printf("OK press %s %d\n", plus ? "+" : "-", n);
  } else if (cmd == "timing") {
    int sp2 = arg.indexOf(' ');
    if (sp2 > 0) {
      press_ms = arg.substring(0, sp2).toInt();
      gap_ms = arg.substring(sp2 + 1).toInt();
    }
    Serial.printf("OK timing press_ms=%d gap_ms=%d\n", press_ms, gap_ms);
  } else {
    Serial.printf("ERR %s unknown\n", cmd.c_str());
  }
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handle(buf);
      buf = "";
    } else if (buf.length() < 64) {
      buf += c;
    }
  }
}
