/*
 * Linux i2c-dev platform layer for the ST VL53L5CX ULD — microduck.
 *
 * Replaces ST's platform.h template (BSD-3-Clause, see LICENSE.txt).
 * Talks to /dev/i2c-N via I2C_RDWR ioctls (../platform.c); used by shim.c,
 * which exposes the flat API
 * `tof::Sensor` wraps.
 *
 * The six hook names below are the callbacks expected by ST's ULD.
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

typedef struct
{
    /* 8-bit I2C address (7-bit << 1), the format the ULD expects. */
    uint16_t address;
    /* Open file descriptor on /dev/i2c-N. */
    int fd;
} VL53L5CX_Platform;

#define VL53L5CX_NB_TARGET_PER_ZONE 1U

/*
 * Output trim: `tof.frame` carries distance + target_status. The target-count
 * block is also required internally: the ULD uses it to replace target_status
 * with 255 in zones where the sensor found no target. The remaining output
 * blocks stay disabled, shrinking each 8x8 readout from ~1.4 kB to ~316 B.
 * Re-enabling another block means checking the shared-bus budget and deciding
 * whether the wire format needs it.
 */
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
/* VL53L5CX_DISABLE_NB_TARGET_DETECTED — kept for status 255 synthesis */
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
/* VL53L5CX_DISABLE_DISTANCE_MM      — kept */
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
/* VL53L5CX_DISABLE_TARGET_STATUS   — kept */
#define VL53L5CX_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
               uint8_t *p_value);
uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
               uint8_t value);
uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
                uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress,
                uint8_t *p_values, uint32_t size);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);

#endif /* _PLATFORM_H_ */
