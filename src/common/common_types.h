/*=========================================================================
 *  Common Type Definitions for Drone-Arm Project
 *
 *  Purpose: Standardize vector and mathematical types across the project
 *  Features:
 *    - FPU-optimized aligned structures
 *    - Consistent type definitions for sensor fusion
 *    - Cross-platform compatibility
 *========================================================================*/

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

/*--------------------------- Vector and Mathematical Types --------------------------*/

/**
 * @brief 3D vector structure optimized for FPU operations
 * @note 16-byte alignment ensures optimal SIMD/FPU vector operations
 */
typedef struct __attribute__((aligned(16))) {
    float x;  /**< X-axis component */
    float y;  /**< Y-axis component */
    float z;  /**< Z-axis component */
} Vec3;

/**
 * @brief Quaternion structure for orientation representation
 * @note Uses standard quaternion notation (w, x, y, z)
 */
typedef struct __attribute__((aligned(16))) {
    float w;  /**< Real/scalar component */
    float x;  /**< i (imaginary) component */
    float y;  /**< j (imaginary) component */
    float z;  /**< k (imaginary) component */
} Quaternion;

/**
 * @brief Euler angles structure (pitch, roll, yaw)
 * @note Units: degrees
 */
typedef struct {
    float pitch;  /**< Pitch angle (rotation around X-axis) */
    float roll;   /**< Roll angle (rotation around Y-axis) */
    float yaw;    /**< Yaw angle (rotation around Z-axis) */
} EulerAngles;

/*--------------------------- Mathematical Constants --------------------------------*/

/**
 * @brief PI constant for mathematical operations
 */
#define PI_FLOAT       3.14159265358979323846f

/**
 * @brief Degrees to radians conversion factor
 */
#define DEG_TO_RAD     (PI_FLOAT / 180.0f)

/**
 * @brief Radians to degrees conversion factor
 */
#define RAD_TO_DEG     (180.0f / PI_FLOAT)

#endif /* COMMON_TYPES_H */