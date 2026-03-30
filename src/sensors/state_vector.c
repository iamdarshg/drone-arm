#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "kernel/scheduler.h"
#include "sensors/state_vector.h"

enum {
    MAX_IMUS = 8u,
    MAX_GPS = 4u,
};

typedef struct {
    bool used;
    uint8_t bus;
    uint8_t addr_or_cs;
    imu_state_vector_t latest;
    bool ready;
} imu_slot_t;

typedef struct {
    bool used;
    uint8_t bus;
    uint8_t addr_or_cs;
    gps_state_vector_t latest;
    bool ready;
} gps_slot_t;

static imu_slot_t g_imus[MAX_IMUS];
static gps_slot_t g_gps[MAX_GPS];
static volatile state_vector_shared_t g_shared;

void state_vector_init(void) {
    uint8_t i;
    (void)memset(g_imus, 0, sizeof(g_imus));
    (void)memset(g_gps, 0, sizeof(g_gps));
    for (i = 0u; i < MAX_IMUS; ++i) {
        g_shared.imu[i] = (imu_state_vector_t){0};
    }
    for (i = 0u; i < MAX_GPS; ++i) {
        g_shared.gps[i] = (gps_state_vector_t){0};
    }
    g_shared.ready = (state_vector_ready_t){0};
}

bool state_vector_register_imu(uint8_t bus, uint8_t addr_or_cs) {
    uint8_t i;
    for (i = 0u; i < MAX_IMUS; ++i) {
        if (!g_imus[i].used) {
            g_imus[i].used = true;
            g_imus[i].bus = bus;
            g_imus[i].addr_or_cs = addr_or_cs;
            g_imus[i].ready = false;
            return true;
        }
    }
    return false;
}

bool state_vector_register_gps(uint8_t bus, uint8_t addr_or_cs) {
    uint8_t i;
    for (i = 0u; i < MAX_GPS; ++i) {
        if (!g_gps[i].used) {
            g_gps[i].used = true;
            g_gps[i].bus = bus;
            g_gps[i].addr_or_cs = addr_or_cs;
            g_gps[i].ready = false;
            return true;
        }
    }
    return false;
}

bool state_vector_request_all_async(void) {
    uint8_t i;
    /* Placeholder async trigger path: mark slots as "in-flight then ready".
     * Real hardware implementation would enqueue DMA bus transactions. */
    for (i = 0u; i < MAX_IMUS; ++i) {
        if (g_imus[i].used) {
            g_imus[i].latest.timestamp_us++;
            g_imus[i].ready = true;
            g_shared.imu[i] = g_imus[i].latest;
            g_shared.ready.imu_ready_mask |= (1u << i);
        }
    }
    for (i = 0u; i < MAX_GPS; ++i) {
        if (g_gps[i].used) {
            g_gps[i].latest.timestamp_us++;
            g_gps[i].ready = true;
            g_shared.gps[i] = g_gps[i].latest;
            g_shared.ready.gps_ready_mask |= (1u << i);
        }
    }
    return true;
}

bool state_vector_poll_ready(state_vector_ready_t *ready) {
    uint8_t i;
    ASSERT(ready != NULL);
    ready->imu_ready_mask = 0u;
    ready->gps_ready_mask = 0u;
    for (i = 0u; i < MAX_IMUS; ++i) {
        if (g_imus[i].used && g_imus[i].ready) {
            ready->imu_ready_mask |= (1u << i);
        }
    }
    for (i = 0u; i < MAX_GPS; ++i) {
        if (g_gps[i].used && g_gps[i].ready) {
            ready->gps_ready_mask |= (1u << i);
        }
    }
    return true;
}

bool state_vector_read_imu(uint8_t index, imu_state_vector_t *out) {
    ASSERT(out != NULL);
    if (index >= MAX_IMUS || !g_imus[index].used || !g_imus[index].ready) {
        return false;
    }
    *out = g_imus[index].latest;
    g_imus[index].ready = false;
    g_shared.ready.imu_ready_mask &= ~(1u << index);
    return true;
}

bool state_vector_read_gps(uint8_t index, gps_state_vector_t *out) {
    ASSERT(out != NULL);
    if (index >= MAX_GPS || !g_gps[index].used || !g_gps[index].ready) {
        return false;
    }
    *out = g_gps[index].latest;
    g_gps[index].ready = false;
    g_shared.ready.gps_ready_mask &= ~(1u << index);
    return true;
}

const volatile state_vector_shared_t *state_vector_shared(void) {
    return &g_shared;
}

bool state_vector_register_shared_region_with_scheduler(void) {
    return scheduler_add_shared_region((uintptr_t)&g_shared, sizeof(g_shared));
}
