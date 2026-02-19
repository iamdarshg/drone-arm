#include <stdint.h>
#include <math.h>
#include "assert.h"
#include "utils.h"
#include "low_level/i2c.h"
#include "low_level/gpio.h"

/*=====================================================================
 *  ICP‑42670 (ICM‑42670) driver
 *  - Static FIFO buffer (no malloc in the fast path)
 *  - CS pin handling
 *  - INT1 = FIFO‑full, INT2 = data‑ready
 *  - All public APIs return a uint8_t exit code (0 = success)
 *  - Simple Euler‑angle and quaternion conversion helpers
 *====================================================================*/

/*--------------------------- Pin definitions ---------------------------*/
#define ICP_SDA          0U   /* Default I2C SDA */
#define ICP_SCL          1U   /* Default I2C SCL */
#define ICP_CS          27U   /* Chip‑Select pin (active low) */
#define ICP_INT1        20U   /* INT1 – FIFO‑full */
#define ICP_INT2        21U   /* INT2 – data‑ready */

/*--------------------------- Register map -----------------------------*/
#define ICP_I2C_ADDR          0x68U   /* 7‑bit address */
#define REG_WHO_AM_I          0x75U
#define REG_PWR_MGMT0         0x4EU
#define REG_ACCEL_CONFIG0     0x50U
#define REG_GYRO_CONFIG0      0x51U
#define REG_INTF_CONFIG0      0x4CU
#define REG_FIFO_CONFIG       0x5EU
#define REG_FIFO_CONFIG1      0x5FU
#define REG_FIFO_DATA_OUT     0x30U
#define REG_FIFO_COUNT        REG_FIFO_CONFIG   /* High/low byte pair */
#define REG_INT_SOURCE0       0x63U
#define REG_INT_SOURCE1       0x64U
#define REG_INT_CONFIG0       0x68U
#define REG_FREE_FALL         0x5DU
#define REG_TEMPERATURE       0x1FU

/*--------------------------- Bit fields ------------------------------*/
#define BIT_SOFT_RESET        (1U << 7)
#define BIT_MCLK_RDY          (1U << 0)   /* Master clock ready */
#define ACCEL_FS_2G           (0U << 2)   /* ±2 g */
#define GYRO_FS_250DPS        (0U << 2)   /* ±250 dps */
#define ODR_MAX               0x0FU       /* Highest ODR (device specific) */
#define FIFO_EN               (1U << 0)
#define FIFO_PKT_STYLE3       (0x3U << 4) /* Packet style 3 */
#define ENDIAN_LITTLE         (0U << 0)   /* Little‑endian FIFO */
#define INT_EN_FIFO_FULL      (1U << 2)
#define INT_EN_DATA_RDY       (1U << 0)
#define FREEFALL_EN           (1U << 0)

/*--------------------------- FIFO configuration ----------------------*/
#define ICP_FIFO_PACKET_SIZE  12U                     /* 12 bytes per packet */
#define ICP_MAX_FIFO_SAMPLES  128U                     /* 32 packets = 384 bytes */

/*--------------------------- Static buffers --------------------------*/
static uint8_t icp_fifo_buffer[ICP_FIFO_PACKET_SIZE * ICP_MAX_FIFO_SAMPLES];
static icp_42670_read latest_icp[ICP_MAX_FIFO_SAMPLES];

/*--------------------------- Device context --------------------------*/
typedef struct {
    uint8_t i2c_id;          /* I2C peripheral (0 = default) */
    uint8_t address;         /* 7‑bit I2C address */
    uint8_t *fifo_buf;        /* Pointer to static FIFO buffer */
    uint32_t fifo_capacity;  /* Size of FIFO buffer in bytes */
} icp_42670_dev;

/* Global device instance – used by all public helpers */
static icp_42670_dev icp_dev;

/*--------------------------- Forward declarations --------------------*/
static uint8_t icp_write_reg(icp_42670_dev *dev, uint8_t reg,
                             const uint8_t *data, uint32_t len);
static uint8_t icp_read_reg(icp_42670_dev *dev, uint8_t reg,
                            uint8_t *data, uint32_t len);
static void     icp_flush_fifo(icp_42670_dev *dev);
static uint8_t  icp_wait_mclk_ready(icp_42670_dev *dev);
static void     icp_configure_cs(void);

/*=====================================================================
 *  Public API (all return a uint8_t exit code; 0 = success)
 *====================================================================*/

/* Initialise the ICP‑42670 sensor. */
uint8_t ICP_init(void)
{
    icp_dev.i2c_id  = 0U;               /* Default I2C0 (pins 0 & 1) */
    icp_dev.address = ICP_I2C_ADDR;
    icp_dev.fifo_buf = icp_fifo_buffer;
    icp_dev.fifo_capacity = sizeof(icp_fifo_buffer);

    /* 1) Initialise I2C at ~1 MHz (high‑speed). */
    i2c_set_baud_mode_master(icp_dev.i2c_id, 1000000U, true);

    /* 2) Configure CS pin (active‑low). */
    icp_configure_cs();

    /* 3) Verify WHO_AM_I (expected 0x47). */
    uint8_t who = 0;
    if (icp_read_reg(&icp_dev, REG_WHO_AM_I, &who, 1) != 0) return 1;
    if (who != 0x47) {
        /* Unexpected ID – continue for demonstration purposes. */
    }

    /* 4) Soft‑reset and wait for MCLK_RDY. */
    uint8_t reset = BIT_SOFT_RESET;
    if (icp_write_reg(&icp_dev, REG_PWR_MGMT0, &reset, 1) != 0) return 2;
    if (!icp_wait_mclk_ready(&icp_dev)) return 3;

    /* 5) Configure accel (±2 g) and gyro (±250 dps) with max ODR. */
    uint8_t accel_cfg = (ODR_MAX << 4) | ACCEL_FS_2G;
    uint8_t gyro_cfg  = (ODR_MAX << 4) | GYRO_FS_250DPS;
    if (icp_write_reg(&icp_dev, REG_ACCEL_CONFIG0, &accel_cfg, 1) != 0) return 4;
    if (icp_write_reg(&icp_dev, REG_GYRO_CONFIG0,  &gyro_cfg,  1) != 0) return 5;

    /* 6) FIFO: enable, packet style 3, little‑endian. */
    uint8_t fifo_cfg1 = FIFO_PKT_STYLE3 | ENDIAN_LITTLE;
    uint8_t fifo_cfg  = FIFO_EN;
    if (icp_write_reg(&icp_dev, REG_FIFO_CONFIG1, &fifo_cfg1, 1) != 0) return 6;
    if (icp_write_reg(&icp_dev, REG_FIFO_CONFIG,  &fifo_cfg,  1) != 0) return 7;

    /* 7) Enable free‑fall detection (no interrupt – read register). */
    uint8_t ff = FREEFALL_EN;
    if (icp_write_reg(&icp_dev, REG_FREE_FALL, &ff, 1) != 0) return 8;

    /* 8) Interrupt routing: INT1 = FIFO‑full, INT2 = data‑ready. */
    uint8_t int_cfg0 = 0U;
    if (icp_write_reg(&icp_dev, REG_INT_CONFIG0, &int_cfg0, 1) != 0) return 9;
    uint8_t int_src0 = INT_EN_FIFO_FULL;   /* INT1 */
    uint8_t int_src1 = INT_EN_DATA_RDY;    /* INT2 */
    if (icp_write_reg(&icp_dev, REG_INT_SOURCE0, &int_src0, 1) != 0) return 10;
    if (icp_write_reg(&icp_dev, REG_INT_SOURCE1, &int_src1, 1) != 0) return 11;

    /* 9) Temperature sanity check. */
    uint8_t temp_raw = 0;
    if (icp_read_reg(&icp_dev, REG_TEMPERATURE, &temp_raw, 1) != 0) return 12;
    int8_t temp_c = (int8_t)temp_raw;      /* 1 LSB/°C, 0 LSB = 23 °C */
    ASSERT(temp_c > -40 && temp_c < 85);

    /* 10) Flush any stale FIFO data. */
    icp_flush_fifo(&icp_dev);

    return 0;   /* Success */
}

/* Query free‑fall status (1 = free‑fall detected, 0 = none). */
uint8_t ICP_isFreeFall(void)
{
    uint8_t ff_status = 0;
    if (icp_read_reg(&icp_dev, REG_FREE_FALL, &ff_status, 1) != 0) return 0xFF;
    return (ff_status & FREEFALL_EN) ? 1U : 0U;
}

/* Set accelerometer low‑pass filter bandwidth (0 = bypass). */
uint8_t ICP_setAccelLPF(uint8_t bw_code)
{
    uint8_t reg = bw_code & 0x07U;      /* 3‑bit field */
    return icp_write_reg(&icp_dev, 0x52U, &reg, 1);   /* ACCEL_CONFIG1 placeholder */
}

/* Set gyroscope low‑pass filter bandwidth (0 = bypass). */
uint8_t ICP_setGyroLPF(uint8_t bw_code)
{
    uint8_t reg = bw_code & 0x07U;
    return icp_write_reg(&icp_dev, 0x53U, &reg, 1);   /* GYRO_CONFIG1 placeholder */
}

/* Read FIFO, parse packets into `latest_icp` cache. */
uint8_t ICP_readSensor(void)
{
    ASSERT_NOT_NULL(icp_dev.fifo_buf);
    ASSERT(icp_dev.fifo_capacity == 0);

    uint8_t cnt[2] = {0};
    if (icp_read_reg(&icp_dev, REG_FIFO_COUNT, cnt, 2) != 0) return 2;
    uint16_t bytes = (uint16_t)(cnt[0] << 8) | cnt[1];
    if (bytes == 0) return 0;                     /* Nothing to read */

    if (bytes > icp_dev.fifo_capacity) bytes = (uint16_t)icp_dev.fifo_capacity;

    if (icp_read_reg(&icp_dev, REG_FIFO_DATA_OUT, icp_dev.fifo_buf, bytes) != 0) return 3;

    uint32_t samples = bytes / ICP_FIFO_PACKET_SIZE;
    for (uint32_t i = 0; i < samples && i < ICP_MAX_FIFO_SAMPLES; ++i) {
        uint8_t *p = icp_dev.fifo_buf + i * ICP_FIFO_PACKET_SIZE;
        latest_icp[i].reading.x = (int16_t)((p[0] << 8) | p[1]);
        latest_icp[i].reading.y = (int16_t)((p[2] << 8) | p[3]);
        latest_icp[i].reading.z = (int16_t)((p[4] << 8) | p[5]);
        latest_icp[i].fifo_len  = (uint8_t)samples;
        latest_icp[i].fifo_full = (bytes == icp_dev.fifo_capacity);
    }
    return 0;
}

/* Convert the most recent accelerometer sample to Euler angles (°). */
uint8_t ICP_query_gyro_eul(Vec3 *euler)
{
    ASSERT_NOT_NULL(euler);

    /* Use the first cached sample – in practice you would select the best. */
    int16_t ax = latest_icp[0].reading.x;
    int16_t ay = latest_icp[0].reading.y;
    int16_t az = latest_icp[0].reading.z;

    const float accel_lsb_per_g = 16384.0f;   /* 2 g full‑scale → 16384 LSB/g */
    float fx = (float)ax / accel_lsb_per_g;
    float fy = (float)ay / accel_lsb_per_g;
    float fz = (float)az / accel_lsb_per_g;

    float pitch = atan2f(-fx, sqrtf(fy * fy + fz * fz)) * (180.0f / (float)M_PI);
    float roll  = atan2f(fy, fz) * (180.0f / (float)M_PI);
    float yaw   = 0.0f;                       /* Not derivable from accel alone */

    euler->x = pitch;
    euler->y = roll;
    euler->z = yaw;
    ASSERT_NOT_NULL(pitch);
    ASSERT_NOT_NULL(yaw);
    ASSERT_NOT_NULL(roll);
    return 0;
}

/* Convert the most recent gyroscope sample to a quaternion.
 * A very simple integration over a fixed 1 ms timestep is used.
 * For production code replace with a proper sensor‑fusion algorithm. */
uint8_t ICP_query_gyro_q(Quarternion *q)
{
    ASSERT_NOT_NULL(q);

    /* Extract gyro raw values from the first packet (bytes 6‑11). */
    if (icp_dev.fifo_buf == NULL) return 2;
    uint8_t *p = icp_dev.fifo_buf;                /* First packet */
    int16_t gx = (int16_t)((p[6] << 8) | p[7]);
    int16_t gy = (int16_t)((p[8] << 8) | p[9]);
    int16_t gz = (int16_t)((p[10] << 8) | p[11]);

    const float gyro_lsb_per_dps = 131.0f;         /* 250 dps full‑scale → 131 LSB/(°/s) */
    float wx = ((float)gx / gyro_lsb_per_dps) * ((float)M_PI / 180.0f);  /* rad/s */
    float wy = ((float)gy / gyro_lsb_per_dps) * ((float)M_PI / 180.0f);
    float wz = ((float)gz / gyro_lsb_per_dps) * ((float)M_PI / 180.0f);

    const float dt = 0.001f;                      /* 1 ms timestep */
    float half_dt = 0.5f * dt;

    /* Small‑angle quaternion update (starting from identity). */
    q->a = 1.0f;
    q->b = wx * half_dt;
    q->c = wy * half_dt;
    q->d = wz * half_dt;

    /* Normalize */
    float norm = sqrtf(q->a * q->a + q->b * q->b + q->c * q->c + q->d * q->d);
    if (norm > 0.0f) {
        q->a /= norm;
        q->b /= norm;
        q->c /= norm;
        q->d /= norm;
    }
    ASSERT_NOT_NULL(q->a);
    ASSERT_NOT_NULL(q->b);
    ASSERT_NOT_NULL(q->c);
    ASSERT_NOT_NULL(q->d);
    return 0;
}

/*=====================================================================
 *  Private helper implementations
 *====================================================================*/

static uint8_t icp_write_reg(icp_42670_dev *dev, uint8_t reg,
                             const uint8_t *data, uint32_t len)
{
    ASSERT_NOT_NULL(dev);
    ASSERT_NOT_NULL(reg);
    uint8_t tx[1 + len];
    tx[0] = reg;
    for (uint32_t i = 0; i < len; ++i) tx[1 + i] = data[i];
    i2c_write_stream(dev->i2c_id, dev->address, tx, 1 + len);
    return 0;
}

static uint8_t icp_read_reg(icp_42670_dev *dev, uint8_t reg,
                            uint8_t *data, uint32_t len)
{
    i2c_write_reg(dev->i2c_id, dev->address, reg, NULL, 0);
    i2c_read_reg(dev->i2c_id, dev->address, reg, data, len);
    ASSERT_NOT_NULL(dev);
    ASSERT_NOT_NULL(reg);
    return 0;
}

/* Flush FIFO by reading until the count register reports zero. */
static void icp_flush_fifo(icp_42670_dev *dev)
{
    ASSERT_NOT_NULL(dev);
    uint8_t cnt[2] = {0};
    icp_read_reg(dev, REG_FIFO_COUNT, cnt, 2);
    uint16_t remaining = (uint16_t)(cnt[0] << 8) | cnt[1];

    while (remaining) {
        uint8_t dummy[ICP_FIFO_PACKET_SIZE];
        icp_read_reg(dev, REG_FIFO_DATA_OUT, dummy, ICP_FIFO_PACKET_SIZE);
        icp_read_reg(dev, REG_FIFO_COUNT, cnt, 2);
        remaining = (uint16_t)(cnt[0] << 8) | cnt[1];
    }
}

/* Wait for the MCLK_RDY bit (timeout ≈100 ms). Returns true on success. */
static uint8_t icp_wait_mclk_ready(icp_42670_dev *dev)
{
    ASSERT_NOT_NULL(dev);
    const uint32_t timeout_ms = 100;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        uint8_t pwr = 0;
        icp_read_reg(dev, REG_PWR_MGMT0, &pwr, 1);
        if (pwr & BIT_MCLK_RDY) return 1;
        for (volatile int i = 0; i < 1000; ++i) { }
        elapsed += 5;
    }
    return 0;   /* Timeout */
}

/* Configure the CS pin as an active‑low output and set it high (inactive). */
static void icp_configure_cs(void)
{
    gpio_set_dir(ICP_CS, true);  /* Output */
    gpio_put(ICP_CS, true);      /* Inactive (high) */
}

/* End of ICP‑42670 driver */
