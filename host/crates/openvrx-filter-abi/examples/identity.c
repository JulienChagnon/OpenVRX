/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * A complete OpenVRX filter plugin in C: copies each field, optionally
 * scaled ("gain=0.5" as the config string). Build:
 *
 *   cc -std=c99 -O2 -shared -fPIC -fvisibility=hidden -I ../include identity.c -o libidentity.so
 *
 * and set the app's video.plugin_path to the library (or pass it to
 * openvrx-cli decode --filter-plugin).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "openvrx_filter.h"

typedef struct {
    float gain;
    char info[128];
} identity_t;

static void *create(const char *config, char *err, size_t err_len) {
    float gain = 1.0f;
    if (config && strncmp(config, "gain=", 5) == 0) {
        gain = (float)atof(config + 5);
    } else if (config && config[0] != '\0') {
        snprintf(err, err_len, "identity: unknown config \"%s\" (want gain=X)", config);
        return NULL;
    }
    identity_t *h = calloc(1, sizeof *h);
    if (!h) {
        snprintf(err, err_len, "identity: out of memory");
        return NULL;
    }
    h->gain = gain;
    snprintf(h->info, sizeof h->info, "{\"name\":\"identity (C)\",\"pipeline\":\"raw\",\"geometry\":\"any\"}");
    return h;
}

static const char *info(void *h) {
    return ((identity_t *)h)->info;
}

static void copy_plane(const float *src, float *dst, uint32_t w, uint32_t h, uint32_t stride, float gain) {
    for (uint32_t r = 0; r < h; r++) {
        for (uint32_t x = 0; x < w; x++) {
            dst[(size_t)r * stride + x] = gain * src[(size_t)r * stride + x];
        }
    }
}

static int32_t process(void *h, const openvrx_field_t *f) {
    float g = ((identity_t *)h)->gain;
    copy_plane(f->y, f->out_y, f->width, f->height, f->stride, g);
    copy_plane(f->u, f->out_u, f->cwidth, f->cheight, f->cstride, g);
    copy_plane(f->v, f->out_v, f->cwidth, f->cheight, f->cstride, g);
    return OPENVRX_PROCESS_WRITTEN;
}

static void destroy(void *h) {
    free(h);
}

static const openvrx_filter_v1 table = {
    OPENVRX_FILTER_ABI, sizeof(openvrx_filter_v1), create, info, process, destroy,
};

OPENVRX_FILTER_EXPORT const openvrx_filter_v1 *openvrx_filter_entry(void) {
    return &table;
}
