#ifndef SENSORS_STATE_VECTOR_H
#define SENSORS_STATE_VECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t accel_mg[3];
    int32_t gyro_mdps[3];
    int32_t mag_mgauss[3];
    int32_t pressure_pa;
    int32_t temperature_cdeg;
    uint32_t timestamp_us;
} imu_state_vector_t;

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_mm;
    int32_t vn_mm_s;
    int32_t ve_mm_s;
    int32_t vd_mm_s;
    uint8_t fix_type;
    uint8_t sats;
    uint16_t hdop_centi;
    uint32_t timestamp_us;
} gps_state_vector_t;

typedef struct {
    uint32_t imu_ready_mask;
    uint32_t gps_ready_mask;
} state_vector_ready_t;

typedef struct {
    volatile imu_state_vector_t imu[8];
    volatile gps_state_vector_t gps[4];
    volatile state_vector_ready_t ready;
} state_vector_shared_t;

void state_vector_init(void);
bool state_vector_register_imu(uint8_t bus, uint8_t addr_or_cs);
bool state_vector_register_gps(uint8_t bus, uint8_t addr_or_cs);
bool state_vector_request_all_async(void);
bool state_vector_poll_ready(state_vector_ready_t *ready);
bool state_vector_read_imu(uint8_t index, imu_state_vector_t *out);
bool state_vector_read_gps(uint8_t index, gps_state_vector_t *out);
const volatile state_vector_shared_t *state_vector_shared(void);
bool state_vector_register_shared_region_with_scheduler(void);

#endif
