// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: att_ctl console over Wi-Fi (see net.h).
#include "net.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>

static const uint16_t PORT = 2323;
static const char *const HOST = "openvrx-bench";  // openvrx-bench.local
static const unsigned TCP_LINE_MAX = 128;  // (LINE_MAX is a libc macro)

static WiFiServer server(PORT);
static WiFiClient client;
static String tcp_buf;
static String ssid, pass;
static bool wifi_on = false;
static bool server_on = false;

// Writes go to USB always and to the TCP client when one is connected. One
// write per reply (printf formats first), so a reply is one TCP segment.
class Tee : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override
  {
    Serial.write(b, n);
    if (client && client.connected()) client.write(b, n);
    return n;
  }
};
static Tee tee;
Print &OUT = tee;

static void save()
{
  Preferences p;
  p.begin("att_ctl", false);
  p.putString("ssid", ssid);
  p.putString("pass", pass);
  p.end();
}

static void wifi_start()
{
  if (ssid.length() == 0) return;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOST);
  WiFi.setSleep(false);         // answers at once instead of on the next beacon
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifi_on = true;
  if (!server_on) {
    server.begin();
    server.setNoDelay(true);
    server_on = true;
  }
}

static void wifi_stop()
{
  if (client) client.stop();
  MDNS.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifi_on = false;
}

void net_begin()
{
  Preferences p;
  p.begin("att_ctl", true);
  ssid = p.getString("ssid", "");
  pass = p.getString("pass", "");
  p.end();
  wifi_start();
}

void net_poll(void (*handle)(String))
{
  static bool was_up = false;
  const bool up = wifi_on && WiFi.status() == WL_CONNECTED;
  if (up && !was_up) {
    OUT.printf("EVT wifi connected ip=%s host=%s.local port=%u\n", WiFi.localIP().toString().c_str(), HOST,
               PORT);
    if (MDNS.begin(HOST)) MDNS.addService("openvrx", "tcp", PORT);
  } else if (!up && was_up) {
    OUT.println("EVT wifi lost");
    MDNS.end();
  }
  was_up = up;
  if (!server_on) return;

  if (server.hasClient()) {
    WiFiClient c = server.accept();
    if (client) client.stop();  // the newest client wins
    client = c;
    client.setNoDelay(true);
    tcp_buf = "";
  }
  while (client && client.connected() && client.available()) {
    char ch = (char)client.read();
    if (ch == '\n' || ch == '\r') {
      if (tcp_buf.length()) {
        String line = tcp_buf;
        tcp_buf = "";
        handle(line);
      }
    } else if (tcp_buf.length() < TCP_LINE_MAX) {
      tcp_buf += ch;
    }
  }
}

bool net_line_pending()
{
  return tcp_buf.length() > 0;
}

static void status(const char *cmd)
{
  const char *st = !wifi_on ? "off" : WiFi.status() == WL_CONNECTED ? "connected" : "connecting";
  OUT.printf("OK %s ssid=%s status=%s", cmd, ssid.length() ? ssid.c_str() : "-", st);
  if (wifi_on && WiFi.status() == WL_CONNECTED)
    OUT.printf(" ip=%s rssi=%d host=%s.local port=%u", WiFi.localIP().toString().c_str(), WiFi.RSSI(), HOST, PORT);
  OUT.println();
}

bool net_command(const String &cmd, const String &arg)
{
  if (cmd == "wifissid") {
    if (arg.length() == 0 || arg.length() > 32) {
      OUT.println("ERR wifissid 1..32 characters");
      return true;
    }
    ssid = arg;
    save();
    OUT.printf("OK wifissid ssid=%s\n", ssid.c_str());
  } else if (cmd == "wifipass") {
    if (arg.length() > 63) {
      OUT.println("ERR wifipass up to 63 characters");
      return true;
    }
    pass = arg;
    save();
    OUT.println("OK wifipass saved");  // never echoed
  } else if (cmd == "wifi") {
    if (arg == "on") {
      if (ssid.length() == 0) {
        OUT.println("ERR wifi no ssid: send wifissid first");
        return true;
      }
      wifi_stop();
      wifi_start();
    } else if (arg == "off") {
      wifi_stop();
    } else if (arg == "forget") {
      wifi_stop();
      ssid = "";
      pass = "";
      save();
    } else if (arg.length()) {
      OUT.println("ERR wifi [on|off|forget]");
      return true;
    }
    status("wifi");
  } else {
    return false;
  }
  return true;
}
