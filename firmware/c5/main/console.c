// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: USB Serial/JTAG console (docs/PROTOCOL.md section 1).
//
// One reader task assembles lines and runs the command handler, so replies
// come out in command order, exactly one per command. All output goes
// through one mutex-protected writer with a bounded wait: with no host
// attached the driver's TX buffer fills and lines are dropped instead of
// blocking the firmware. ESP-IDF log output is captured with
// esp_log_set_vprintf and re-emitted as "LOG " lines.
#include "console.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "openvrx_cmd.h"

#define OUT_LINE_MAX 512u
#define WRITE_TIMEOUT_MS 50u
#define READER_STACK 6144u
#define READER_PRIO 5u

static SemaphoreHandle_t s_out_lock;
static StaticSemaphore_t s_out_lock_storage;
static void (*s_handler)(char *line);
static char s_log_line[OUT_LINE_MAX];
static size_t s_log_len;

static void write_raw(const char *data, size_t len)
{
    size_t done = 0;
    while (done < len) {
        const int n = usb_serial_jtag_write_bytes(data + done, len - done, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
        if (n <= 0) return;   // no host draining the port: drop the rest
        done += (size_t)n;
    }
}

static void write_line_locked(const char *prefix, const char *text, size_t len)
{
    char buf[OUT_LINE_MAX + 8];
    size_t n = 0;
    const size_t plen = strlen(prefix);
    memcpy(buf, prefix, plen);
    n = plen;
    if (len > OUT_LINE_MAX) len = OUT_LINE_MAX;
    memcpy(buf + n, text, len);
    n += len;
    buf[n++] = '\n';
    write_raw(buf, n);
}

static void write_line(const char *prefix, const char *text)
{
    if (!s_out_lock) return;
    xSemaphoreTake(s_out_lock, portMAX_DELAY);
    write_line_locked(prefix, text, strlen(text));
    xSemaphoreGive(s_out_lock);
}

void console_reply(const char *line)
{
    write_line("", line);
}

static void vformat_line(const char *prefix, const char *fmt, va_list ap)
{
    char text[OUT_LINE_MAX];
    vsnprintf(text, sizeof(text), fmt, ap);
    write_line(prefix, text);
}

void console_evt(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vformat_line("EVT ", fmt, ap);
    va_end(ap);
}

void console_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vformat_line("LOG ", fmt, ap);
    va_end(ap);
}

void console_flush(void)
{
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(200));
}

// ESP-IDF log sink: collects fragments until a newline, then emits one
// "LOG " line. Calls from interrupt context are dropped (never block there).
static int log_vprintf(const char *fmt, va_list ap)
{
    if (xPortInIsrContext() || !s_out_lock ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        return 0;
    char frag[OUT_LINE_MAX];
    const int n = vsnprintf(frag, sizeof(frag), fmt, ap);
    if (n <= 0) return n;
    size_t len = (size_t)n < sizeof(frag) ? (size_t)n : sizeof(frag) - 1u;
    if (xSemaphoreTake(s_out_lock, pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) != pdTRUE) return 0;
    for (size_t k = 0; k < len; ++k) {
        const char c = frag[k];
        if (c == '\n') {
            if (s_log_len) write_line_locked("LOG ", s_log_line, s_log_len);
            s_log_len = 0;
        } else if (c != '\r' && s_log_len < sizeof(s_log_line)) {
            s_log_line[s_log_len++] = c;
        }
    }
    xSemaphoreGive(s_out_lock);
    return n;
}

static void reader_task(void *arg)
{
    (void)arg;
    static openvrx_linebuf_t lb;
    openvrx_linebuf_init(&lb);
    uint8_t chunk[64];
    for (;;) {
        const int n = usb_serial_jtag_read_bytes(chunk, sizeof(chunk), portMAX_DELAY);
        for (int k = 0; k < n; ++k) {
            const openvrx_line_status_t st = openvrx_linebuf_feed(&lb, (char)chunk[k]);
            if (st == OPENVRX_LINE_OVERFLOW) {
                console_reply("ERR line syntax line longer than 120 characters");
            } else if (st == OPENVRX_LINE_READY) {
                // An empty line carries no command: no reply.
                bool blank = true;
                for (const char *p = lb.buf; *p; ++p) blank &= (*p == ' ' || *p == '\t');
                if (!blank) s_handler(lb.buf);
            }
        }
    }
}

esp_err_t console_start(void (*handler)(char *line))
{
    s_handler = handler;
    s_out_lock = xSemaphoreCreateMutexStatic(&s_out_lock_storage);
    usb_serial_jtag_driver_config_t cfg = {.tx_buffer_size = 4096, .rx_buffer_size = 1024};
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) return err;
    // stdout/stderr (panic handler excepted) now share the driver's buffer.
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdout, NULL, _IONBF, 0);
    esp_log_set_vprintf(log_vprintf);
    if (xTaskCreate(reader_task, "console", READER_STACK, NULL, READER_PRIO, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}
