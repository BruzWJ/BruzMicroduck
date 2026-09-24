/*
 * Linux i2c-dev platform layer for ST's VL53L5CX ULD.
 *
 * The platform struct type is supplied by the build (see ../build.rs):
 *
 *   TOF_PLATFORM   the platform struct type the driver's header defines
 * ST ships a template with these six hooks left to the integrator; this is the
 * Linux one, carried over from `microduck_runtime` where it was measured. Direct
 * I2C_RDWR ioctls, chunked. Doing the transfers in C rather than through a
 * per-callback trip into a scripting language cuts the per-transaction overhead
 * ~10x: the 90 kB firmware upload takes a few seconds at 400 kHz instead of
 * tens of seconds — and that upload happens on every sensor start.
 */

#include "platform.h"

/* The ULD's platform struct, under one local name for the code below. */
typedef TOF_PLATFORM Platform;

#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sched.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>

/* One chunk per I2C_RDWR message. At 400 kHz, 256 data bytes hold the shared
 * Qwiic bus for about 6 ms. That leaves the 50 Hz body-IMU loop a scheduling
 * opportunity during the VL53L5CX's ~90 kB firmware upload instead of one
 * 2 KiB transfer occupying the adapter for roughly 46 ms. */
#define CHUNK 256u

static uint8_t wr_buf[CHUNK + 2];

static uint8_t xfer(Platform *p, struct i2c_msg *msgs, int n)
{
    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = (uint32_t)n };
    /* I2C_RDWR returns the number of messages completed. A short positive
     * result is still a failed register transaction; accepting it would hand
     * the ULD a partially updated buffer as if it were a valid response. */
    return ioctl(p->fd, I2C_RDWR, &data) == n ? 0 : 255;
}

uint8_t RdMulti(Platform *p, uint16_t reg, uint8_t *values,
                uint32_t size)
{
    uint32_t off = 0;
    while (off < size) {
        uint16_t idx = (uint16_t)(reg + off);
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        uint8_t idx_buf[2] = { (uint8_t)(idx >> 8), (uint8_t)idx };
        struct i2c_msg msgs[2] = {
            { .addr = (uint16_t)(p->address >> 1), .flags = 0,
              .len = 2, .buf = idx_buf },
            { .addr = (uint16_t)(p->address >> 1), .flags = I2C_M_RD,
              .len = (uint16_t)n, .buf = values + off },
        };
        if (xfer(p, msgs, 2)) return 255;
        off += n;
        if (off < size) sched_yield();
    }
    return 0;
}

uint8_t WrMulti(Platform *p, uint16_t reg, uint8_t *values,
                uint32_t size)
{
    uint32_t off = 0;
    while (off < size) {
        uint16_t idx = (uint16_t)(reg + off);
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        wr_buf[0] = (uint8_t)(idx >> 8);
        wr_buf[1] = (uint8_t)idx;
        memcpy(wr_buf + 2, values + off, n);
        struct i2c_msg msg = {
            .addr = (uint16_t)(p->address >> 1), .flags = 0,
            .len = (uint16_t)(n + 2), .buf = wr_buf,
        };
        if (xfer(p, &msg, 1)) return 255;
        off += n;
        if (off < size) sched_yield();
    }
    return 0;
}

uint8_t RdByte(Platform *p, uint16_t reg, uint8_t *value)
{
    return RdMulti(p, reg, value, 1);
}

uint8_t WrByte(Platform *p, uint16_t reg, uint8_t value)
{
    return WrMulti(p, reg, &value, 1);
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    /* Byte-reverse each 32-bit word in place (size is a multiple of 4). */
    for (uint16_t i = 0; i + 3 < size; i += 4) {
        uint8_t a = buffer[i], b = buffer[i + 1];
        buffer[i] = buffer[i + 3];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = b;
        buffer[i + 3] = a;
    }
}

uint8_t WaitMs(Platform *p, uint32_t ms)
{
    (void)p;
    struct timespec ts = { .tv_sec = ms / 1000,
                           .tv_nsec = (long)(ms % 1000) * 1000000L };
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
    return 0;
}
