#include "crc.h"

uint8_t crc8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0; for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i]; for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
        }
    } return crc;
}

uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF; for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i]; for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
        }
    } return crc;
}

uint32_t crc32(const uint8_t *data, uint16_t len) {
    uint32_t crc = 0xFFFFFFFF; for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i]; for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
    } return ~crc;
}
