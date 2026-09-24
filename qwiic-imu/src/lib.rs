//! The SparkFun LSM6DSV16X Qwiic IMUs used in the trunk and head.
//!
//! Both breakout sizes carry the same chip and expose the same two addresses.
//! The replica keeps roles fixed rather than probing and guessing: the body
//! Micro board stays at its factory `0x6b`, while the head board's address
//! jumper selects `0x6a`. [`Sensor`] configures ST's on-chip Sensor Fusion Low
//! Power (SFLP) algorithm and returns SI-unit samples in the chip's own axes.
//! Both boards are installed +X forward, +Y left, +Z up; consumers still own
//! frame placement because the head articulates while the trunk does not.
//!
//! The hardware implementation is Linux-only (`/dev/i2c-*`). Off Linux the
//! same API refuses to open, so fake/simulated daemons still compile without a
//! pretend sensor.

/// Factory address of either SparkFun board; assigned to the body IMU.
pub const BODY_ADDRESS: u8 = 0x6b;
/// Alternate address selected by the SparkFun ADDR jumper; assigned to the head IMU.
pub const HEAD_ADDRESS: u8 = 0x6a;

/// One fused sample, in the LSM6DSV16X's own sensor axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Monotonic count of valid game-rotation vectors produced by this sensor.
    pub sequence: u64,
    /// Angular velocity, rad/s.
    pub gyro: [f32; 3],
    /// Specific force, m/s².
    pub accel: [f32; 3],
    /// Sensor→world game rotation, scalar-first `[w, x, y, z]`.
    pub quat: [f32; 4],
    /// Die temperature, °C.
    pub temp_c: f32,
}

/// One hardware poll. `sample` is absent when SFLP has not produced a new
/// quaternion since the preceding poll; that is freshness information, not an
/// I²C error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Poll {
    pub sample: Option<Sample>,
}

/// The nearest supported sensor rate at or above a consumer's requested poll
/// rate. SFLP has discrete 15/30/60/120/240/480 Hz rungs.
pub fn supported_rate(requested_hz: u16) -> u16 {
    match requested_hz {
        0..=15 => 15,
        16..=30 => 30,
        31..=60 => 60,
        61..=120 => 120,
        121..=240 => 240,
        _ => 480,
    }
}

/// Decode the three half-precision SFLP components into scalar-first form.
///
/// SFLP omits `w`; game rotation chooses its non-negative root. A tagged FIFO
/// record with zero `x/y/z` is a valid identity rotation. Unlike the old
/// register bridge, FIFO presence itself tells the caller that this is a fresh
/// result, so zero must not be repurposed as an initialization sentinel.
fn game_rotation(raw: [u16; 3]) -> Option<[f32; 4]> {
    let mut xyz = raw.map(half_to_f32);
    if !xyz.iter().all(|v| v.is_finite()) {
        return None;
    }
    let mut norm_sq = xyz.iter().map(|v| v * v).sum::<f32>();
    // Half precision can put a valid unit quaternion just above one. A larger
    // excess is a corrupt FIFO record, not something to normalise into truth.
    if norm_sq > 1.02 {
        return None;
    }
    if norm_sq > 1.0 {
        let norm = norm_sq.sqrt();
        xyz.iter_mut().for_each(|v| *v /= norm);
        norm_sq = 1.0;
    }
    Some([(1.0 - norm_sq).sqrt(), xyz[0], xyz[1], xyz[2]])
}

fn half_to_f32(bits: u16) -> f32 {
    let sign = if bits & 0x8000 == 0 { 1.0 } else { -1.0 };
    let exp = ((bits >> 10) & 0x1f) as i32;
    let frac = (bits & 0x03ff) as f32;
    match exp {
        0 => sign * frac * 2.0_f32.powi(-24),
        0x1f if frac == 0.0 => sign * f32::INFINITY,
        0x1f => f32::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2.0_f32.powi(exp - 15),
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use std::time::{Duration, Instant};

    use anyhow::{Context, Result, anyhow, bail};
    use embedded_hal::delay::DelayNs;
    use linux_embedded_hal::I2cdev;
    use lsm6dsv16x_rs::blocking::{Lsm6dsv16x, prelude::*};
    use st_mems_bus::blocking::I2cBus;

    use super::{Poll, Sample, game_rotation, supported_rate};

    type Driver = Lsm6dsv16x<I2cBus<I2cdev>, StdDelay, MainBank>;

    const RESET_TIMEOUT: Duration = Duration::from_millis(100);
    const MAX_FIFO_RECORDS: u16 = 64;
    const GRAVITY: f32 = 9.806_65;
    const GYRO_RAD_PER_COUNT: f32 = 17.5e-3 * std::f32::consts::PI / 180.0;
    const ACCEL_MPS2_PER_COUNT: f32 = 0.122e-3 * GRAVITY;
    const TEMP_EVERY: u64 = 100;

    #[derive(Debug, Clone, Copy)]
    struct StdDelay;

    impl DelayNs for StdDelay {
        fn delay_ns(&mut self, ns: u32) {
            std::thread::sleep(Duration::from_nanos(u64::from(ns)));
        }
    }

    struct Rates {
        hz: u16,
        raw: Odr,
        sflp: SflpDataRate,
    }

    fn rates(requested_hz: u16) -> Rates {
        match supported_rate(requested_hz) {
            15 => Rates {
                hz: 15,
                raw: Odr::_15hz,
                sflp: SflpDataRate::_15hz,
            },
            30 => Rates {
                hz: 30,
                raw: Odr::_30hz,
                sflp: SflpDataRate::_30hz,
            },
            60 => Rates {
                hz: 60,
                raw: Odr::_60hz,
                sflp: SflpDataRate::_60hz,
            },
            120 => Rates {
                hz: 120,
                raw: Odr::_120hz,
                sflp: SflpDataRate::_120hz,
            },
            240 => Rates {
                hz: 240,
                raw: Odr::_240hz,
                sflp: SflpDataRate::_240hz,
            },
            _ => Rates {
                hz: 480,
                raw: Odr::_480hz,
                sflp: SflpDataRate::_480hz,
            },
        }
    }

    /// One open LSM6DSV16X. It owns its own i2c-dev descriptor; Linux
    /// serialises its transactions with the other Qwiic devices on the adapter.
    pub struct Sensor {
        driver: Driver,
        rate_hz: u16,
        sequence: u64,
        next_temp_sequence: u64,
        temp_c: f32,
    }

    impl Sensor {
        pub fn open(bus: &std::path::Path, address: u8, requested_hz: u16) -> Result<Self> {
            if !matches!(address, 0x6a | 0x6b) {
                bail!("LSM6DSV16X address must be 0x6a or 0x6b, got {address:#04x}");
            }
            let i2c = I2cdev::new(bus).with_context(|| format!("open {}", bus.display()))?;
            let i2c_address = if address == 0x6a {
                I2CAddress::I2cAddL
            } else {
                I2CAddress::I2cAddH
            };
            let mut driver = Lsm6dsv16x::new_i2c(i2c, i2c_address, StdDelay);
            driver.tim.delay_ms(5);

            let id = driver
                .device_id_get()
                .map_err(|e| anyhow!("read WHO_AM_I at {address:#04x}: {e:?}"))?;
            if id != ID {
                bail!("device at {address:#04x} has WHO_AM_I {id:#04x}, expected {ID:#04x}");
            }

            driver
                // Match ST's sensor-fusion bring-up: a power-on reset clears
                // both the ordinary registers and the embedded SFLP state.
                // A control-register-only reset can leave fusion state behind
                // when a daemon reopens an already powered sensor.
                .reset_set(Reset::GlobalRst)
                .map_err(|e| anyhow!("reset LSM6DSV16X: {e:?}"))?;
            let deadline = Instant::now() + RESET_TIMEOUT;
            loop {
                let state = driver
                    .reset_get()
                    .map_err(|e| anyhow!("wait for LSM6DSV16X reset: {e:?}"))?;
                if state == Reset::Ready {
                    break;
                }
                if Instant::now() >= deadline {
                    bail!("LSM6DSV16X reset did not finish within {RESET_TIMEOUT:?}");
                }
                driver.tim.delay_ms(1);
            }

            driver
                .block_data_update_set(1)
                .map_err(|e| anyhow!("enable block-data update: {e:?}"))?;
            driver
                .xl_full_scale_set(XlFullScale::_4g)
                .map_err(|e| anyhow!("set accelerometer range: {e:?}"))?;
            driver
                .gy_full_scale_set(GyFullScale::_500dps)
                .map_err(|e| anyhow!("set gyroscope range: {e:?}"))?;

            let rates = rates(requested_hz);
            driver
                .fifo_sflp_batch_set(FifoSflpRaw {
                    game_rotation: 1,
                    gravity: 0,
                    gbias: 0,
                })
                .map_err(|e| anyhow!("batch SFLP game rotation: {e:?}"))?;
            driver
                .fifo_mode_set(FifoMode::StreamMode)
                .map_err(|e| anyhow!("start IMU FIFO: {e:?}"))?;
            driver
                .xl_data_rate_set(rates.raw)
                .map_err(|e| anyhow!("set accelerometer rate: {e:?}"))?;
            driver
                .gy_data_rate_set(rates.raw)
                .map_err(|e| anyhow!("set gyroscope rate: {e:?}"))?;
            driver
                .sflp_data_rate_set(rates.sflp)
                .map_err(|e| anyhow!("set SFLP rate: {e:?}"))?;
            driver
                .sflp_game_rotation_set(1)
                .map_err(|e| anyhow!("enable SFLP game rotation: {e:?}"))?;
            // SFLP estimates gyro bias automatically while the device is
            // stationary. `sflp_game_gbias_set` is the optional path for
            // restoring a bias persisted by an application; calling it with
            // zero adds no information beyond reset defaults, and the 2.1.0
            // upstream implementation contains unbounded hardware-status
            // loops. Omit it until a real persisted bias and bounded driver
            // support both exist.

            Ok(Self {
                driver,
                rate_hz: rates.hz,
                sequence: 0,
                next_temp_sequence: 1,
                temp_c: 25.0,
            })
        }

        pub fn rate_hz(&self) -> u16 {
            self.rate_hz
        }

        pub fn poll(&mut self) -> Result<Poll> {
            let status = self
                .driver
                .fifo_status_get()
                .map_err(|e| anyhow!("read IMU FIFO status: {e:?}"))?;
            if status.fifo_ovr != 0 || status.fifo_level > MAX_FIFO_RECORDS {
                // Do not spend an unbounded control tick draining old records.
                // Resetting stream mode keeps the live sensor configuration and
                // makes the next poll start at current data.
                self.driver
                    .fifo_mode_set(FifoMode::BypassMode)
                    .map_err(|e| anyhow!("reset overflowing IMU FIFO: {e:?}"))?;
                self.driver
                    .fifo_mode_set(FifoMode::StreamMode)
                    .map_err(|e| anyhow!("restart IMU FIFO: {e:?}"))?;
                bail!(
                    "IMU FIFO backlog is {} records (overrun={}); discarded it",
                    status.fifo_level,
                    status.fifo_ovr
                );
            }

            let mut newest = None;
            for _ in 0..status.fifo_level {
                let record = self
                    .driver
                    .fifo_out_raw_get()
                    .map_err(|e| anyhow!("read IMU FIFO record: {e:?}"))?;
                if record.tag != Tag::SflpGameRotationVectorTag {
                    continue;
                }
                let raw = [
                    u16::from_le_bytes([record.data[0], record.data[1]]),
                    u16::from_le_bytes([record.data[2], record.data[3]]),
                    u16::from_le_bytes([record.data[4], record.data[5]]),
                ];
                if let Some(quat) = game_rotation(raw) {
                    self.sequence = self.sequence.saturating_add(1);
                    newest = Some((self.sequence, quat));
                }
            }

            let Some((sequence, quat)) = newest else {
                return Ok(Poll { sample: None });
            };
            let gyro_raw = self
                .driver
                .angular_rate_raw_get()
                .map_err(|e| anyhow!("read gyroscope: {e:?}"))?;
            let accel_raw = self
                .driver
                .acceleration_raw_get()
                .map_err(|e| anyhow!("read accelerometer: {e:?}"))?;
            if sequence >= self.next_temp_sequence {
                let raw = self
                    .driver
                    .temperature_raw_get()
                    .map_err(|e| anyhow!("read IMU temperature: {e:?}"))?;
                self.temp_c = f32::from(raw) / 256.0 + 25.0;
                // `sequence` can jump when one poll drains multiple FIFO
                // records. A deadline cannot be skipped the way `% 100 == 0`
                // can when every observed sequence has the wrong parity.
                self.next_temp_sequence = sequence.saturating_add(TEMP_EVERY);
            }

            Ok(Poll {
                sample: Some(Sample {
                    sequence,
                    gyro: gyro_raw.map(|v| f32::from(v) * GYRO_RAD_PER_COUNT),
                    accel: accel_raw.map(|v| f32::from(v) * ACCEL_MPS2_PER_COUNT),
                    quat,
                    temp_c: self.temp_c,
                }),
            })
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::Sensor;

#[cfg(not(target_os = "linux"))]
pub struct Sensor(std::convert::Infallible);

#[cfg(not(target_os = "linux"))]
impl Sensor {
    pub fn open(bus: &std::path::Path, _address: u8, _requested_hz: u16) -> anyhow::Result<Self> {
        anyhow::bail!(
            "no i2c-dev on this platform, so {} cannot be opened",
            bus.display()
        )
    }

    pub fn rate_hz(&self) -> u16 {
        match self.0 {}
    }

    pub fn poll(&mut self) -> anyhow::Result<Poll> {
        match self.0 {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_rounds_up_to_an_sflp_rung() {
        assert_eq!(supported_rate(1), 15);
        assert_eq!(supported_rate(15), 15);
        assert_eq!(supported_rate(16), 30);
        assert_eq!(supported_rate(50), 60);
        assert_eq!(supported_rate(100), 120);
        assert_eq!(supported_rate(255), 480);
        assert_eq!(supported_rate(u16::MAX), 480);
    }

    #[test]
    fn half_precision_decodes_known_values() {
        assert_eq!(half_to_f32(0x0000), 0.0);
        assert_eq!(half_to_f32(0x3c00), 1.0);
        assert_eq!(half_to_f32(0xbc00), -1.0);
        assert_eq!(half_to_f32(0x3800), 0.5);
        assert!((half_to_f32(0x3555) - 0.333).abs() < 1e-3);
        assert_eq!(half_to_f32(0x0001), 2.0_f32.powi(-24));
    }

    #[test]
    fn game_rotation_is_scalar_first_and_accepts_identity() {
        assert_eq!(game_rotation([0; 3]), Some([1.0, 0.0, 0.0, 0.0]));
        // x=0.5, y=z=0 gives w=sqrt(3)/2.
        let q = game_rotation([0x3800, 0, 0]).expect("valid quaternion");
        assert!((q[0] - 0.75_f32.sqrt()).abs() < 1e-6);
        assert_eq!(&q[1..], &[0.5, 0.0, 0.0]);
    }

    #[test]
    fn corrupt_game_rotation_is_not_normalised_into_a_measurement() {
        // Three ones have a squared norm of three.
        assert_eq!(game_rotation([0x3c00; 3]), None);
        assert_eq!(game_rotation([0x7e00, 0, 0]), None, "NaN");
    }
}
