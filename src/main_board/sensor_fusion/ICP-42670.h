/*=========================================================================
 *  ICP-42670 (ICM-42670) P10-Compliant Hardware FPU-Accelerated Driver Header
 *
 *  Interface: I2C with DMA support
 *  Pins: 0=SDA, 1=SCL, 27=CS, 20=INT1, 21=INT2 (configurable in .c file)
 *
 *  Features:
 *  - DMA coprocessor acceleration for bulk I2C transfers (>= 16 bytes)
 *  - Hardware floating-point unit (FPU) with explicit ARM assembly
 *  - SIMD-aligned data structures for optimal FPU access
 *  - Protected I2C transactions with timeout handling
 *  - Conditional DMA usage based on transfer size
 *
 *  P10 Compliance Features:
 *  - Explicit bounds checking
 *  - Const correctness throughout
 *  - No undefined behavior
 *  - Defensive programming assertions
 *  - Magic number elimination via named constants
 *========================================================================*/

#ifndef ICP_42670_H
#define ICP_42670_H

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

    uint8_t fifo_len;   /**< Number of samples in FIFO */
    uint8_t fifo_full;  /**< FIFO full flag (1=full, 0=not full) */
    uint8_t padding[6];  /**< Padding for 16-byte alignment */
} icp_42670_read_aligned;

/*--------------------------- Public API --------------------------------*/

/**
 * @brief Initialize ICP-42670 sensor with DMA and hardware FPU
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 I2C initialization failed
 * @retval 2 WHO_AM_I verification failed (not 0x67)
 * @retval 3 Software reset failed
 * @retval 4 MCLK_RDY timeout
 * @retval 5 Accelerometer configuration failed
 * @retval 6 Gyroscope configuration failed
 * @retval 7 FIFO_CONFIG1 write failed
 * @retval 8 FIFO_CONFIG write failed
 * @retval 9 Free-fall configuration failed
 * @retval 10 Interrupt config0 write failed
 * @retval 11 INT_SOURCE0 write failed
 * @retval 12 INT_SOURCE1 write failed
 * @retval 13 Temperature sensor read failed (low byte)
 * @retval 14 Temperature sensor read failed (high byte)
 * @retval 15 Temperature out of range (-40 to 85°C)
 */
uint8_t ICP_init(void);

/**
 * @brief Query free-fall detection status
 * @return 1 if free-fall detected, 0 if not, 0xFF on error
 * @retval 0 Free-fall not detected
 * @retval 1 Free-fall detected
 * @retval 0xFF Register read error
 */
uint8_t ICP_isFreeFall(void);

/**
 * @brief Set accelerometer low-pass filter bandwidth
 * @param bw_code 3-bit bandwidth code (0 = bypass)
 * @return 0 on success, non-zero on error
 * @retval 0 Success
 * @retval 1 Register write failed
 */
uint8_t ICP_setAccelLPF(uint8_t bw_code);

/**
 * @brief Set gyroscope low-pass filter bandwidth
 * @param bw_code 3-bit bandwidth code (0 = bypass)
 * @return 0 on success, non-zero on error
 * @retval 0 Success
 * @retval 1 Register write failed
 */
uint8_t ICP_setGyroLPF(uint8_t bw_code);

/**
 * @brief Read FIFO data using conditional DMA based on transfer size
 * @details Automatically uses DMA for transfers >= 16 bytes, blocking I2C otherwise
 * @return 0 on success, non-zero error code
 * @retval 0 Success
 * @retval 1 FIFO_COUNTH register read failed
 * @retval 2 FIFO_COUNTL register read failed
 * @retval 3 FIFO data read failed
 */
uint8_t ICP_readSensor(void);

/**
 * @brief Convert accelerometer data to Euler angles using FPU assembly
 * @param euler Output vector for pitch, roll, yaw (float, units: degrees)
 * @return 0 on success
 * @retval 0 Success
 * @retval 1 Null pointer provided
 */
uint8_t ICP_query_gyro_eul(Vec3 *euler);

/**
 * @brief Convert gyroscope data to quaternion using FPU assembly
 * @param q Output quaternion (normalized, float components)
 * @return 0 on success
 * @retval 0 Success
 * @retval 1 Null pointer provided
 * @retval 2 FIFO buffer not initialized
 */
uint8_t ICP_query_gyro_q(Quaternion *q);

/*--------------------------- Constants for User Reference ------------*/

/**
 * @brief WHO_AM_I value for ICM-42670-P
 */
#define ICP_WHO_AM_I_VALUE      0x67U

/**
 * @brief Temperature conversion formula: Temp(°C) = (raw / 132.48) + 25
 */
#define ICP_TEMP_CONVERSION(raw)  (((float)(raw) / 132.48f) + 25.0f)

/**
 * @brief FIFO packet size in bytes
 */
#define ICP_FIFO_PACKET_SIZE      12U  /* 12 bytes per sample */

/**
 * @brief Maximum FIFO samples to process
 */
#define ICP_MAX_FIFO_SAMPLES      128U

#endif /* ICP_42670_H */
