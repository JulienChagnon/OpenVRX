/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * openvrx_filter.h: OpenVRX video filter plugin ABI, version 1.
 *
 * A filter plugin is a shared library (.so, .dll, .dylib) that the OpenVRX
 * app ("plugin" video preset) and openvrx-cli (decode --filter-plugin) load
 * at run time. The host hands it every displayed field of a view as float
 * planes and shows what it writes back. The same types exist as Rust types
 * in the openvrx-filter-abi crate; both describe one ABI and are checked
 * against each other by that crate's tests.
 *
 * ENTRY POINT
 *   The library exports one symbol with C linkage:
 *       const openvrx_filter_v1 *openvrx_filter_entry(void);
 *   It returns a pointer to a table that stays valid for the life of the
 *   process (a static). The host reads abi and size first and refuses the
 *   library unless abi == OPENVRX_FILTER_ABI, size >= sizeof(openvrx_filter_v1)
 *   and all four function pointers are set. A later revision of version 1
 *   may append members; it then reports a larger size.
 *
 * THREADING
 *   Every call into one library (create, info, process and destroy, for
 *   every handle it creates) comes from one host thread, one call at a
 *   time. The host keeps that thread for the life of the process, so a
 *   plugin needs no locking and may keep thread-bound state (a GPU context
 *   made in create is current in process). Two views running the same
 *   library share that thread; their calls are interleaved, never
 *   concurrent.
 *
 * LIFETIMES
 *   create(config, err, err_len)
 *     config is a NUL-terminated UTF-8 string (the host's plugin_config
 *     setting, possibly empty), valid during the call only. On success
 *     return a non-NULL handle. On failure return NULL and write a
 *     NUL-terminated message into err (err_len bytes, truncate to fit).
 *     create may take long (loading a model, building an engine): the
 *     host shows the plain picture meanwhile.
 *   info(h)
 *     Returns a NUL-terminated UTF-8 JSON object owned by the plugin,
 *     valid until destroy(h). The host reads it once, right after create.
 *   process(h, f)
 *     f and everything it points to (planes, aux array, names) are valid
 *     during the call only; keep no pointers. Input planes are read-only.
 *     The out planes are host memory with the same layout as the inputs,
 *     not aliasing them; their content on entry is undefined.
 *   destroy(h)
 *     Called once for every handle create returned, on the same thread.
 *     No call uses h afterwards.
 *
 * RETURN VALUES OF process
 *   0  (OPENVRX_PROCESS_WRITTEN)      out_y, out_u and out_v are written:
 *                                     the host shows them.
 *   1  (OPENVRX_PROCESS_NOT_WRITTEN)  nothing written: the host shows the
 *                                     input field (for example while a
 *                                     temporal filter fills its history).
 *   <0 error: the host stops calling process on this handle, destroys it,
 *      falls back to the plain picture and shows the code on its OSD. The
 *      plugin may print details on stderr.
 *
 * FAILURE
 *   A plugin runs inside the host process: a crash ends the host. Never let
 *   a C++ exception or a Rust panic unwind out of these functions.
 *
 * PLANES
 *   Y is luma, 0 (black) .. 1 (white); noise and overshoot go a little
 *   past both ends. U and V are the BT.601 colour differences
 *   U = 0.492 (B - Y), V = 0.877 (R - Y) (about +-0.436 and +-0.615 at full
 *   saturation). Strides count floats, not bytes: row r of a plane starts at
 *   data + r * stride. The host of ABI 1 always passes chroma at full
 *   resolution (cwidth == width, cheight == height) and the picture a view
 *   shows after each field: a full-height frame per field (NTSC 754 x 480,
 *   PAL 922 x 576 on the app's sample grid) from the video pipeline the
 *   plugin asks for. With OPENVRX_FIELD_STEADY_OSD each field line fills two
 *   rows on the same rows for both fields (the goggles' steady OSD);
 *   otherwise each field sits at its true half-line position with the rows
 *   between its lines interpolated.
 *
 * FIELD METADATA
 *   field_index  counts the fields the host produced for this handle,
 *                from 0, one per field. A jump of more than one means the
 *                host skipped fields because the plugin was still busy; the
 *                field after a skip carries OPENVRX_FIELD_DISCONTINUITY.
 *   parity       0: top field (its lines are frame rows 0, 2, 4 ...),
 *                1: bottom field.
 *   flags        OPENVRX_FIELD_DISCONTINUITY: the fields before this one are
 *                unrelated to it (first field, retune, sync relock, new
 *                standard or size, pipeline restart, skipped fields): start
 *                any temporal history again from this field.
 *                OPENVRX_FIELD_STEADY_OSD: line-doubled geometry (above).
 *   cn_db        carrier-to-noise ratio of the field in dB as the receiver
 *                measured it (median over its rows), NaN when unknown.
 *   aux          the side planes the plugin named in its info JSON, in that
 *                order (n_aux of them, aux NULL when none). Host side
 *                planes of ABI 1:
 *                  "damage"          width x height: the receiver's damage
 *                                    mask, 0 clean .. 1 no data (mask byte
 *                                    / 255: noisy 64, repaired click 96,
 *                                    click 160, carrier gone 224, lost 255).
 *                  "row_cn_db"       1 x height (width 1, stride 1), per
 *                  "row_noise_ire"   picture row: the line's C/N in dB,
 *                  "row_sync_score"  noise on the porch in IRE, sync score
 *                  "row_timing_err"  0..1, timing error in grid samples,
 *                  "row_clicks"      FM clicks, and its line flag bits as a
 *                  "row_flags"       number. Unknown values are NaN or
 *                                    infinite, as the receiver reports them.
 *
 * INFO JSON (keys the host reads, all optional; others are ignored)
 *   "name"      shown on the OSD (default: the library file name).
 *   "pipeline"  video preset the plugin expects as its input: "raw"
 *               (default), "low_latency", "balanced" or "quality".
 *   "geometry"  "steady": the host feeds line-doubled fields whatever its
 *               steady OSD setting; "fields": true half-line positions;
 *               "any" (default): follows the setting (see the flags).
 *   "aux"       array of side plane names (above); the host refuses a
 *               plugin that asks for one it does not have.
 *   "provider"  short text for the OSD, such as the compute device.
 *
 * DEADLINE (app)
 *   The app hands each field over as soon as it is complete and waits at
 *   most its plugin_deadline_ms setting (default 6 ms) for the result; then
 *   it shows the field unfiltered. If the plugin is still busy with an
 *   earlier field when the next one is complete, that field is shown
 *   unfiltered and waits in a one-field mailbox (a newer field replaces a
 *   waiting one, which is then skipped). The plugin so keeps getting fields
 *   in order when it runs late; its output is shown only when on time.
 *
 * LOADING
 *   The host loads a library once per process and never unloads it (GPU
 *   runtimes leave threads behind): restart the app after rebuilding one.
 */
#ifndef OPENVRX_FILTER_H
#define OPENVRX_FILTER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OPENVRX_FILTER_ABI 1u

/* Name of the entry point, for hosts that look it up. */
#define OPENVRX_FILTER_ENTRY_NAME "openvrx_filter_entry"

/* Field flags. */
#define OPENVRX_FIELD_DISCONTINUITY 1u /* history before this field is unrelated (retune, relock, standard change, first field) */
#define OPENVRX_FIELD_STEADY_OSD 2u    /* line-doubled steady-OSD geometry */

/* Return values of process (negative: error). */
#define OPENVRX_PROCESS_WRITTEN 0
#define OPENVRX_PROCESS_NOT_WRITTEN 1

/* A named side plane. stride in floats. */
typedef struct {
    const char *name;
    uint32_t width, height, stride;
    const float *data;
} openvrx_plane_t;

/* One field. */
typedef struct {
    uint32_t width, height, stride;    /* luma plane, stride in floats */
    uint32_t cwidth, cheight, cstride; /* chroma planes */
    const float *y, *u, *v;            /* live scale: Y 0..1, BT.601 U/V */
    float *out_y, *out_u, *out_v;      /* same layout, written by the plugin */
    uint64_t field_index;              /* +1 per field fed */
    uint32_t parity;                   /* 0 top, 1 bottom */
    uint32_t flags;                    /* OPENVRX_FIELD_* */
    float cn_db;                       /* NaN when unknown */
    uint32_t n_aux;
    const openvrx_plane_t *aux;        /* optional named side planes, may be 0/NULL */
} openvrx_field_t;

/* The table openvrx_filter_entry returns. */
typedef struct {
    uint32_t abi;  /* OPENVRX_FILTER_ABI */
    uint32_t size; /* sizeof(openvrx_filter_v1) */
    void *(*create)(const char *config, char *err, size_t err_len); /* NULL on error, message in err */
    const char *(*info)(void *h);                                   /* JSON, valid until destroy */
    int32_t (*process)(void *h, const openvrx_field_t *f);          /* 0 = out written, 1 = not written (show input), <0 error */
    void (*destroy)(void *h);
} openvrx_filter_v1;

typedef const openvrx_filter_v1 *(*openvrx_filter_entry_fn)(void);

/* Put OPENVRX_FILTER_EXPORT on the plugin's definition of
 * openvrx_filter_entry so it is exported (also with -fvisibility=hidden). */
#if defined(_WIN32)
#define OPENVRX_FILTER_EXPORT __declspec(dllexport)
#elif defined(__GNUC__)
#define OPENVRX_FILTER_EXPORT __attribute__((visibility("default")))
#else
#define OPENVRX_FILTER_EXPORT
#endif

/* The one exported symbol. */
const openvrx_filter_v1 *openvrx_filter_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENVRX_FILTER_H */
