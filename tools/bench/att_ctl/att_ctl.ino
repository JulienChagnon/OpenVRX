// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: presses the AW8637 step attenuator's + and - buttons.
//
// Board: any ESP32 (tested target: ESP-32S NodeMCU), Arduino core.
// Wiring (docs/BENCH.md): PIN_PLUS and PIN_MINUS each drive one photorelay
// or NPN transistor whose output is wired across that button. Common GND.
//
// The attenuator has no readback, so the position is counted here:
//   1. Set the attenuator's step size to 0.25 dB once by hand (press + and -
//      together until the 0.25 dB step is selected; it is remembered).
//   2. "home" presses - enough times to reach 0 dB from anywhere.
//   3. "set X" presses + or - to reach X dB (0 to 31.75, 0.25 dB grid).
//
// Serial 115200, one command per line, one "OK <cmd> ..." or "ERR <cmd> ..."
// reply each (same style as the C5 console):
//   home | set <dB> | get | press <+|-> [n] | timing <press_ms> <gap_ms>

#include <Arduino.h>

static const int PIN_PLUS = 26;
static const int PIN_MINUS = 27;
// Low-level-trigger relay modules switch on when the input is LOW.
static const bool ACTIVE_LOW = true;
static const int STEPS_MAX = 127;  // 31.75 dB / 0.25 dB

static int press_ms = 60;
static int gap_ms = 80;
static int position = -1;  // in 0.25 dB steps, -1 = unknown until "home"

static void drive(int pin, bool on) {
  digitalWrite(pin, (on != ACTIVE_LOW) ? HIGH : LOW);
}

static void press(int pin, int n) {
  for (int k = 0; k < n; k++) {
    drive(pin, true);
    delay(press_ms);
    drive(pin, false);
    delay(gap_ms);
  }
}

void setup() {
  pinMode(PIN_PLUS, OUTPUT);
  pinMode(PIN_MINUS, OUTPUT);
  drive(PIN_PLUS, false);
  drive(PIN_MINUS, false);
  Serial.begin(115200);
  Serial.println("att_ctl ready (send 'home' first)");
}

static void handle(String line) {
  line.trim();
  if (line.length() == 0) return;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? "" : line.substring(sp + 1);
  arg.trim();

  if (cmd == "home") {
    press(PIN_MINUS, STEPS_MAX + 3);
    position = 0;
    Serial.println("OK home db=0");
  } else if (cmd == "set") {
    if (position < 0) {
      Serial.println("ERR set not homed");
      return;
    }
    float db = arg.toFloat();
    int want = (int)lroundf(db * 4.0f);
    if (want < 0 || want > STEPS_MAX) {
      Serial.println("ERR set range 0..31.75");
      return;
    }
    if (want > position) press(PIN_PLUS, want - position);
    if (want < position) press(PIN_MINUS, position - want);
    position = want;
    Serial.printf("OK set db=%.2f\n", position / 4.0f);
  } else if (cmd == "get") {
    if (position < 0) Serial.println("OK get db=unknown");
    else Serial.printf("OK get db=%.2f\n", position / 4.0f);
  } else if (cmd == "press") {
    int n = 1;
    int sp2 = arg.indexOf(' ');
    if (sp2 > 0) n = arg.substring(sp2 + 1).toInt();
    bool plus = arg.startsWith("+");
    press(plus ? PIN_PLUS : PIN_MINUS, n);
    if (position >= 0) position = constrain(position + (plus ? n : -n), 0, STEPS_MAX);
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
