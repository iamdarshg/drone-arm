# External Sensor Requirements (Summary)

This document summarizes the requirements for the external sensors based on their datasheets.

## 1. ICM-42670-P (6-Axis IMU)
- **Interface**: SPI (up to 24 MHz) or I2C (up to 1 MHz).
- **Supply Voltage**: 1.71V to 3.6V.
- **Features**: 3-axis accelerometer and 3-axis gyroscope.
- **Register WHO_AM_I**: Default value is `0x67`.

## 2. LSM6DSO32 (6-Axis IMU)
- **Interface**: SPI (up to 10 MHz) or I2C (up to 1 MHz).
- **Supply Voltage**: 1.71V to 3.6V.
- **Features**: 3-axis accelerometer and 3-axis gyroscope, 32g full scale.
- **Register WHO_AM_I**: Default value is `0x6C`.

## 3. ICP-42670-P (Pressure Sensor)
- **Interface**: I2C (up to 1 MHz) or SPI (up to 10 MHz).
- **Supply Voltage**: 1.71V to 3.6V.
- **Range**: 30 to 110 kPa.
- **Accuracy**: ±1 Pa.
