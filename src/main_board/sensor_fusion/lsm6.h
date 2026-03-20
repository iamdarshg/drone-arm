/*=========================================================================
 *  LSM6DSO32 P10-Compliant Hardware FPU-Accelerated Driver Header
 *
 *  Interface: SPI with DMA support
 *  Pins: 8=MISO, 9=SCK, 10=MOSI, 11=CS (configurable in .c file)
 *
 *  Features:
 *  - DMA coprocessor acceleration for bulk transfers (>= 16 bytes)
 *  - Hardware floating-point unit (FPU) with explicit ARM assembly
 *  - SIMD-aligned data structures for optimal FPU/DMA performance
 *  - Atomic chip-select handling for RTOS-safe operation
 *  - Conditional DMA usage based on transfer size threshold
 *
 *  P10 Compliance Features:
 *  - Explicit bounds checking
 *  - Const correctness throughout
 *  - No undefined behavior
 *  - Defensive programming assertions
 *  - Magic number elimination via named constants
 *  - Include guards for safe multi-file inclusion
  *========================================================================*/

#ifndef LSM6_H
#define LSM6_H

#include <stdint.h>
#include "../common/common_types.h"

/*--------------------------- Type Definitions --------------------------*/

/* Use Vec3 and Quaternion from common_types.h */



/**
 * @brief Aligned sensor read structure for DMA/FPU efficiency
 * @note 16-byte alignment ensures optimal FPU vector operations
 */
typedef struct __attribute__((aligned(16))) {
    struct __attribute__((aligned(16))) {
        float x;  /**< Accelerometer X-axis (g) */
        float y;  /**< Accelerometer Y-axis (g) */
        float z;  /**< Accelerometer Z-axis (g) */
    } reading;  /**< Accelerometer data */

    struct __attribute__((aligned(16))) {
        float x;  /**< Gyroscope X-axis (dps) */
        float y;  /**< Gyroscope Y-axis (dps) */
        float z;  /**< Gyroscope Z-axis (dps) */
    } angular_rate;  /**< Gyroscope data in degrees per second */

    float temperature;  /**< Temperature (°C) */
    uint8_t fifo_len;   /**< Number of samples in FIFO */
    uint8_t fifo_full;  /**< FIFO full flag (1=full, 0=not full) */
    uint8_t padding[6];  /**< Padding for 16-byte alignment */
} lsm6_read_aligned;

/*--------------------------- Public API --------------------------------*/

/**
 * @brief Initialize LSM6DSO32 sensor with SPI, DMA, and hardware FPU
 * @param spi_id SPI peripheral ID (0 = SPI0, 1 = SPI1, etc.)
 * @param cs_pin Chip select pin (active low)
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 SPI initialization failed
 * @retval 2 WHO_AM_I verification failed
 * @retval 3 Accelerometer configuration failed
 * @retval 4 Gyroscope configuration failed
 * @retval 5 Control register 3 configuration failed
 * @retval 6 FIFO configuration failed
 * @retval 7 INT1 configuration failed
 * @retval 8 Temperature sensor read failed
 * @retval 9 Temperature sensor read failed (high byte)
 * @retval 10 Temperature out of range
 * @retval 11 FIFO watermark configuration failed
 */
uint8_t LSM6_init(uint8_t spi_id);

/**
 * @brief Set accelerometer full-scale range
 * @param fs_sel Full-scale selection:
 *               - 0 = ±2g
 *               - 1 = ±4g
 *               - 2 = ±8g
 *               - 3 = ±16g
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 Invalid parameter (fs_sel > 3)
 * @retval 2 Register read failed
 * @retval 3 Register write failed
 */
uint8_t LSM6_setAccelFullScale(uint8_t fs_sel);

/**
 * @brief Set gyroscope full-scale range
 * @param fs_sel Full-scale selection:
 *               - 0 = ±125 dps
 *               - 1 = ±250 dps
 *               - 2 = ±500 dps
 *               - 3 = ±1000 dps
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 Invalid parameter (fs_sel > 3)
 * @retval 2 Register read failed
 * @retval 3 Register write failed
 */
uint8_t LSM6_setGyroFullScale(uint8_t fs_sel);

/**
 * @brief Read FIFO data using conditional DMA based on transfer size
 * @details Automatically uses DMA for transfers >= 16 bytes, blocking SPI otherwise
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 FIFO status register read failed (low byte)
 * @retval 2 FIFO status register read failed (high byte)
 * @retval 3 Temperature sensor read failed (low byte)
 * @retval 4 Temperature sensor read failed (high byte)
 * @retval 5 Accelerometer data read failed
 * @retval 6 Gyroscope data read failed
 */
uint8_t LSM6_readSensor(void);

/**
 * @brief Convert accelerometer data to Euler angles using FPU assembly
 * @param euler Output vector for pitch, roll, yaw (units: degrees)
 * @return 0 on success
 * @retval 0 Success
 * @retval 1 Null pointer provided
 */
uint8_t LSM6_query_accel_eul(Vec3 *euler);

/**
 * @brief Convert gyroscope data to quaternion using FPU assembly
 * @param q Output quaternion (normalized, float components)
 * @return 0 on success
 * @retval 0 Success
 * @retval 1 Null pointer provided
 */
uint8_t LSM6_query_gyro_q(Quaternion *q);

/*--------------------------- Constants for User Reference ------------*/

/**
 * @brief Accelerometer full-scale selection constants
 */
#define ACCEL_FSSEL_2G      0U  /**< ±2g full-scale range */
#define ACCEL_FSSEL_4G      1U  /**< ±4g full-scale range */
#define ACCEL_FSSEL_8G      2U  /**< ±8g full-scale range */
#define ACCEL_FSSEL_16G     3U  /**< ±16g full-scale range */

/**
 * @brief Gyroscope full-scale selection constants
 */
#define GYRO_FSSEL_125DPS   0U  /**< ±125 dps full-scale range */
#define GYRO_FSSEL_250DPS   1U  /**< ±250 dps full-scale range */
#define GYRO_FSSEL_500DPS   2U  /**< ±500 dps full-scale range */
#define GYRO_FSSEL_1000DPS  3U  /**< ±1000 dps full-scale range */

#endif /* LSM6_H */
