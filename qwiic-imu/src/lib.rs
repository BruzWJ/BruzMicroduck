//! Explicit LSM6DSO and LSM6DSV16X support for Qwiic-connected IMUs.
//!
//! A caller selects the chip model; address probing never changes the driver.
//! LSM6DSV16X orientation comes from ST's on-chip Sensor Fusion Low Power
//! (SFLP) engine. LSM6DSO has no orientation engine, so every paired FIFO
//! accel/gyro sample passes through the same host-side Fusion AHRS session and
//! only the newest fused result from a poll is returned. All public vectors are
//! SI values in the chip's own axes, and quaternions are scalar-first sensor to
//! world rotations.
//!
//! The hardware implementation is Linux-only (`/dev/i2c-*`). Off Linux the
//! same API refuses to open so simulated daemons still compile without
//! inventing sensor data.

use std::fmt;
use std::str::FromStr;

use serde::{Deserialize, Serialize};

mod lsm6dso;

/// Factory address assigned to the body IMU by the reference wiring.
pub const BODY_ADDRESS: u8 = 0x6b;
/// Alternate address assigned to the head IMU by the reference wiring.
pub const HEAD_ADDRESS: u8 = 0x6a;

/// IMU silicon selected by configuration.
///
/// The serialized names are also the only accepted command-line spellings.
/// There is deliberately no address-based detection or compatibility alias.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Model {
    Lsm6dso,
    Lsm6dsv16x,
}

impl Model {
    pub const ALL: [Self; 2] = [Self::Lsm6dso, Self::Lsm6dsv16x];
    pub const LABELS: [&'static str; 2] = [Self::Lsm6dso.as_str(), Self::Lsm6dsv16x.as_str()];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Lsm6dso => "lsm6dso",
            Self::Lsm6dsv16x => "lsm6dsv16x",
        }
    }
}

impl fmt::Display for Model {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl FromStr for Model {
    type Err = anyhow::Error;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|model| model.as_str() == value)
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "unknown IMU model {value:?}; expected {}",
                    Self::LABELS.join(" or ")
                )
            })
    }
}

/// One fused reading in the selected chip's sensor axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Monotonic count of complete source measurements from this sensor.
    pub sequence: u64,
    /// Raw angular velocity, rad/s.
    ///
    /// Software bias correction, when required by the selected model, is used
    /// only by its orientation filter so both backends expose the same signal.
    pub gyro: [f32; 3],
    /// Specific force, m/s².
    pub accel: [f32; 3],
    /// Sensor→world game rotation, scalar-first `[w, x, y, z]`.
    pub quat: [f32; 4],
    /// Die temperature, °C.
    pub temp_c: f32,
}

/// Result of one bounded hardware poll.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Poll {
    /// Newest ready fused sample, if one was produced.
    pub sample: Option<Sample>,
    /// The selected sensor produced a complete usable source measurement.
    ///
    /// This remains true for complete LSM6DSO accel/gyro pairs while its host
    /// orientation and bias session is warming, when `sample` is intentionally
    /// absent. False means there was no fresh usable measurement and lets a
    /// caller distinguish convergence from a malformed or frozen FIFO.
    pub observed: bool,
}

const GRAVITY: f32 = 9.806_65;
const GYRO_RAD_PER_COUNT: f32 = 17.5e-3 * std::f32::consts::PI / 180.0;
const ACCEL_MPS2_PER_COUNT: f32 = 0.122e-3 * GRAVITY;

fn dsv_supported_rate(requested_hz: u16) -> u16 {
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
/// SFLP omits `w`; game rotation chooses its non-negative root. A tagged FIFO record with zero
/// `x/y/z` is a valid identity rotation, not an initialization sentinel.
fn game_rotation(raw: [u16; 3]) -> Option<[f32; 4]> {
    let mut xyz = raw.map(half_to_f32);
    if !xyz.iter().all(|value| value.is_finite()) {
        return None;
    }
    let mut norm_sq = xyz.iter().map(|value| value * value).sum::<f32>();
    // Half precision can put a valid unit quaternion just above one. A larger
    // excess is a corrupt FIFO record, not something to normalise into truth.
    if norm_sq > 1.02 {
        return None;
    }
    if norm_sq > 1.0 {
        let norm = norm_sq.sqrt();
        xyz.iter_mut().for_each(|value| *value /= norm);
        norm_sq = 1.0;
    }
    Some([(1.0 - norm_sq).sqrt(), xyz[0], xyz[1], xyz[2]])
}

fn half_to_f32(bits: u16) -> f32 {
    let sign = if bits & 0x8000 == 0 { 1.0 } else { -1.0 };
    let exponent = ((bits >> 10) & 0x1f) as i32;
    let fraction = (bits & 0x03ff) as f32;
    match exponent {
        0 => sign * fraction * 2.0_f32.powi(-24),
        0x1f if fraction == 0.0 => sign * f32::INFINITY,
        0x1f => f32::NAN,
        _ => sign * (1.0 + fraction / 1024.0) * 2.0_f32.powi(exponent - 15),
    }
}

fn sflp_game_rotation(is_game_rotation: bool, data: [u8; 6]) -> Option<[f32; 4]> {
    if !is_game_rotation {
        return None;
    }
    game_rotation([
        u16::from_le_bytes([data[0], data[1]]),
        u16::from_le_bytes([data[2], data[3]]),
        u16::from_le_bytes([data[4], data[5]]),
    ])
}

#[cfg(target_os = "linux")]
mod linux {
    use std::time::{Duration, Instant};

    use anyhow::{Context, Result, anyhow, bail};
    use embedded_hal::delay::DelayNs;
    use fusion_ahrs::{Ahrs, AhrsSettings, Bias, BiasSettings, Convention};
    use linux_embedded_hal::I2cdev;
    use lsm6dsv16x_rs::blocking::{Lsm6dsv16x, prelude::*};
    use st_mems_bus::blocking::I2cBus;

    use super::{
        ACCEL_MPS2_PER_COUNT, GRAVITY, GYRO_RAD_PER_COUNT, Model, Poll, Sample, dsv_supported_rate,
        lsm6dso, sflp_game_rotation,
    };

    type DsvDriver = Lsm6dsv16x<I2cBus<I2cdev>, StdDelay, MainBank>;

    const RESET_TIMEOUT: Duration = Duration::from_millis(100);
    const MAX_FIFO_RECORDS: u16 = 64;
    const TEMP_EVERY: u64 = 100;

    const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;
    const FUSION_GAIN: f32 = 0.5;
    const ACCEL_REJECTION_DEG: f32 = 10.0;
    const REJECTION_TIMEOUT_S: f32 = 5.0;
    const GYRO_RANGE_DPS: f32 = 500.0;

    #[derive(Debug, Clone, Copy)]
    struct StdDelay;

    impl DelayNs for StdDelay {
        fn delay_ns(&mut self, ns: u32) {
            std::thread::sleep(Duration::from_nanos(u64::from(ns)));
        }
    }

    struct DsvRates {
        hz: f32,
        raw: Odr,
        sflp: SflpDataRate,
    }

    fn dsv_rates(requested_hz: u16) -> DsvRates {
        match dsv_supported_rate(requested_hz) {
            15 => DsvRates {
                hz: 15.0,
                raw: Odr::_15hz,
                sflp: SflpDataRate::_15hz,
            },
            30 => DsvRates {
                hz: 30.0,
                raw: Odr::_30hz,
                sflp: SflpDataRate::_30hz,
            },
            60 => DsvRates {
                hz: 60.0,
                raw: Odr::_60hz,
                sflp: SflpDataRate::_60hz,
            },
            120 => DsvRates {
                hz: 120.0,
                raw: Odr::_120hz,
                sflp: SflpDataRate::_120hz,
            },
            240 => DsvRates {
                hz: 240.0,
                raw: Odr::_240hz,
                sflp: SflpDataRate::_240hz,
            },
            _ => DsvRates {
                hz: 480.0,
                raw: Odr::_480hz,
                sflp: SflpDataRate::_480hz,
            },
        }
    }

    struct DsvSensor {
        driver: DsvDriver,
        rate_hz: f32,
        sequence: u64,
        next_temp_sequence: u64,
        temp_c: f32,
    }

    impl DsvSensor {
        fn open(bus: &std::path::Path, address: u8, requested_hz: u16) -> Result<Self> {
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
                .map_err(|error| anyhow!("read WHO_AM_I at {address:#04x}: {error:?}"))?;
            if id != ID {
                bail!(
                    "device at {address:#04x} has WHO_AM_I {id:#04x}, expected {ID:#04x} for LSM6DSV16X"
                );
            }

            // A global reset clears both ordinary registers and embedded SFLP state. A
            // control-register-only reset could retain fusion state when a daemon reopens an
            // already powered sensor.
            driver
                .reset_set(Reset::GlobalRst)
                .map_err(|error| anyhow!("reset LSM6DSV16X: {error:?}"))?;
            let deadline = Instant::now() + RESET_TIMEOUT;
            loop {
                let state = driver
                    .reset_get()
                    .map_err(|error| anyhow!("wait for LSM6DSV16X reset: {error:?}"))?;
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
                .map_err(|error| anyhow!("enable block-data update: {error:?}"))?;
            driver
                .xl_full_scale_set(XlFullScale::_4g)
                .map_err(|error| anyhow!("set accelerometer range: {error:?}"))?;
            driver
                .gy_full_scale_set(GyFullScale::_500dps)
                .map_err(|error| anyhow!("set gyroscope range: {error:?}"))?;

            let rates = dsv_rates(requested_hz);
            driver
                .fifo_sflp_batch_set(FifoSflpRaw {
                    game_rotation: 1,
                    gravity: 0,
                    gbias: 0,
                })
                .map_err(|error| anyhow!("batch SFLP game rotation: {error:?}"))?;
            driver
                .fifo_mode_set(FifoMode::StreamMode)
                .map_err(|error| anyhow!("start IMU FIFO: {error:?}"))?;
            driver
                .xl_data_rate_set(rates.raw)
                .map_err(|error| anyhow!("set accelerometer rate: {error:?}"))?;
            driver
                .gy_data_rate_set(rates.raw)
                .map_err(|error| anyhow!("set gyroscope rate: {error:?}"))?;
            driver
                .sflp_data_rate_set(rates.sflp)
                .map_err(|error| anyhow!("set SFLP rate: {error:?}"))?;
            driver
                .sflp_game_rotation_set(1)
                .map_err(|error| anyhow!("enable SFLP game rotation: {error:?}"))?;
            // SFLP estimates gyro bias while stationary. The driver's optional setter is for
            // restoring an application-persisted bias; supplying an invented zero adds no
            // information, and upstream 2.1.0 can wait without a bound in that path.

            Ok(Self {
                driver,
                rate_hz: rates.hz,
                sequence: 0,
                next_temp_sequence: 1,
                temp_c: 25.0,
            })
        }

        fn poll(&mut self) -> Result<Poll> {
            let status = self
                .driver
                .fifo_status_get()
                .map_err(|error| anyhow!("read LSM6DSV16X FIFO status: {error:?}"))?;
            if status.fifo_ovr != 0 || status.fifo_level > MAX_FIFO_RECORDS {
                // Bound one control tick instead of draining old history. Toggling FIFO mode
                // retains the live sensor configuration and resumes with current data.
                self.driver
                    .fifo_mode_set(FifoMode::BypassMode)
                    .map_err(|error| anyhow!("reset overflowing LSM6DSV16X FIFO: {error:?}"))?;
                self.driver
                    .fifo_mode_set(FifoMode::StreamMode)
                    .map_err(|error| anyhow!("restart LSM6DSV16X FIFO: {error:?}"))?;
                bail!(
                    "LSM6DSV16X FIFO backlog is {} records (overrun={}); discarded it",
                    status.fifo_level,
                    status.fifo_ovr
                );
            }

            let mut newest = None;
            for _ in 0..status.fifo_level {
                let record = self
                    .driver
                    .fifo_out_raw_get()
                    .map_err(|error| anyhow!("read LSM6DSV16X FIFO record: {error:?}"))?;
                if let Some(quat) =
                    sflp_game_rotation(record.tag == Tag::SflpGameRotationVectorTag, record.data)
                {
                    self.sequence = self.sequence.saturating_add(1);
                    newest = Some((self.sequence, quat));
                }
            }

            let Some((sequence, quat)) = newest else {
                return Ok(Poll {
                    sample: None,
                    // Wrong-tag and corrupt SFLP records are not freshness.
                    // Otherwise they could keep a held orientation alive.
                    observed: false,
                });
            };
            let gyro_raw = self
                .driver
                .angular_rate_raw_get()
                .map_err(|error| anyhow!("read LSM6DSV16X gyroscope: {error:?}"))?;
            let accel_raw = self
                .driver
                .acceleration_raw_get()
                .map_err(|error| anyhow!("read LSM6DSV16X accelerometer: {error:?}"))?;
            if sequence >= self.next_temp_sequence {
                let raw = self
                    .driver
                    .temperature_raw_get()
                    .map_err(|error| anyhow!("read LSM6DSV16X temperature: {error:?}"))?;
                self.temp_c = f32::from(raw) / 256.0 + 25.0;
                // A poll may drain several records, so a deadline cannot be skipped the way a
                // sequence modulus can when every observed sequence has the wrong parity.
                self.next_temp_sequence = sequence.saturating_add(TEMP_EVERY);
            }
            Ok(Poll {
                sample: Some(Sample {
                    sequence,
                    gyro: gyro_raw.map(|value| f32::from(value) * GYRO_RAD_PER_COUNT),
                    accel: accel_raw.map(|value| f32::from(value) * ACCEL_MPS2_PER_COUNT),
                    quat,
                    temp_c: self.temp_c,
                }),
                observed: true,
            })
        }
    }

    pub(crate) struct DsoFusion {
        ahrs: Ahrs,
        bias: Bias,
        bias_acquired: bool,
    }

    impl DsoFusion {
        pub(crate) fn new(rate_hz: f32) -> Result<Self> {
            if !rate_hz.is_finite() || rate_hz <= 0.0 {
                bail!("invalid LSM6DSO sample rate {rate_hz}");
            }
            Ok(Self {
                ahrs: Ahrs::with_settings(AhrsSettings {
                    sample_rate: rate_hz,
                    convention: Convention::Nwu,
                    gain: FUSION_GAIN,
                    gyroscope_range: GYRO_RANGE_DPS,
                    acceleration_rejection: ACCEL_REJECTION_DEG,
                    magnetic_rejection: 0.0,
                    rejection_timeout: REJECTION_TIMEOUT_S,
                }),
                bias: Bias::with_settings(BiasSettings {
                    sample_rate: rate_hz,
                    ..BiasSettings::default()
                }),
                bias_acquired: false,
            })
        }

        pub(crate) fn restart(&mut self) {
            self.ahrs.restart();
            self.bias.restart();
            self.bias_acquired = false;
        }

        pub(crate) fn ready(&self) -> bool {
            !self.ahrs.flags().startup && self.bias_acquired
        }

        pub(crate) fn update(&mut self, raw: lsm6dso::RawSample, temp_c: f32) -> Option<Sample> {
            if !raw
                .gyro
                .iter()
                .chain(&raw.accel)
                .all(|value| value.is_finite())
            {
                self.restart();
                return None;
            }

            let corrected = self.bias.update(raw.gyro.map(|value| value * RAD_TO_DEG));
            // `is_active` means the configured stationary interval has elapsed.
            // It becomes false on later motion, so latch the acquisition for the
            // lifetime of this uninterrupted fusion session.
            self.bias_acquired |= self.bias.is_active();
            self.ahrs
                .update_no_magnetometer(corrected, raw.accel.map(|value| value / GRAVITY));
            if !self.ready() {
                return None;
            }

            let quat: [f32; 4] = self.ahrs.quaternion().into();
            let norm_sq = quat.iter().map(|value| value * value).sum::<f32>();
            if !quat.iter().all(|value| value.is_finite()) || !(0.98..=1.02).contains(&norm_sq) {
                self.restart();
                return None;
            }

            Some(Sample {
                sequence: raw.sequence,
                gyro: raw.gyro,
                accel: raw.accel,
                quat,
                temp_c,
            })
        }
    }

    struct DsoSensor {
        raw: lsm6dso::Sensor,
        fusion: DsoFusion,
    }

    impl DsoSensor {
        fn open(bus: &std::path::Path, address: u8, requested_hz: u16) -> Result<Self> {
            let raw = lsm6dso::Sensor::open(bus, address, requested_hz)?;
            let fusion = DsoFusion::new(raw.rate_hz())?;
            Ok(Self { raw, fusion })
        }

        fn poll(&mut self) -> Result<Poll> {
            let batch = match self.raw.poll() {
                Ok(batch) => batch,
                Err(error) => {
                    // A missing interval cannot be reconstructed. Any raw I²C
                    // or FIFO error starts a new bias/orientation session.
                    self.fusion.restart();
                    return Err(error.context("poll LSM6DSO raw FIFO"));
                }
            };

            let observed = !batch.samples.is_empty();
            let mut newest = None;
            for raw in batch.samples {
                if let Some(sample) = self.fusion.update(raw, batch.temp_c) {
                    newest = Some(sample);
                }
            }
            Ok(Poll {
                sample: newest,
                observed,
            })
        }
    }

    enum Backend {
        Lsm6dso(Box<DsoSensor>),
        Lsm6dsv16x(DsvSensor),
    }

    /// One explicitly selected IMU session.
    pub struct Sensor {
        backend: Backend,
    }

    impl Sensor {
        pub fn open(
            bus: &std::path::Path,
            address: u8,
            model: Model,
            requested_hz: u16,
        ) -> Result<Self> {
            let backend = match model {
                Model::Lsm6dso => {
                    Backend::Lsm6dso(Box::new(DsoSensor::open(bus, address, requested_hz)?))
                }
                Model::Lsm6dsv16x => {
                    Backend::Lsm6dsv16x(DsvSensor::open(bus, address, requested_hz)?)
                }
            };
            Ok(Self { backend })
        }

        pub fn model(&self) -> Model {
            match &self.backend {
                Backend::Lsm6dso(_) => Model::Lsm6dso,
                Backend::Lsm6dsv16x(_) => Model::Lsm6dsv16x,
            }
        }

        pub fn rate_hz(&self) -> f32 {
            match &self.backend {
                Backend::Lsm6dso(sensor) => sensor.raw.rate_hz(),
                Backend::Lsm6dsv16x(sensor) => sensor.rate_hz,
            }
        }

        /// Whether orientation output is ready for publication.
        ///
        /// LSM6DSO becomes ready after both the host AHRS startup interval and
        /// one stationary gyro-bias acquisition; LSM6DSV16X becomes ready
        /// after its first valid SFLP vector.
        pub fn ready(&self) -> bool {
            match &self.backend {
                Backend::Lsm6dso(sensor) => sensor.fusion.ready(),
                Backend::Lsm6dsv16x(sensor) => sensor.sequence != 0,
            }
        }

        pub fn poll(&mut self) -> Result<Poll> {
            match &mut self.backend {
                Backend::Lsm6dso(sensor) => sensor.poll(),
                Backend::Lsm6dsv16x(sensor) => sensor.poll(),
            }
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::Sensor;

#[cfg(not(target_os = "linux"))]
pub struct Sensor(std::convert::Infallible);

#[cfg(not(target_os = "linux"))]
impl Sensor {
    pub fn open(
        bus: &std::path::Path,
        _address: u8,
        _model: Model,
        _requested_hz: u16,
    ) -> anyhow::Result<Self> {
        anyhow::bail!(
            "no i2c-dev on this platform, so an IMU on {} cannot be opened",
            bus.display()
        )
    }

    pub fn model(&self) -> Model {
        match self.0 {}
    }

    pub fn rate_hz(&self) -> f32 {
        match self.0 {}
    }

    pub fn ready(&self) -> bool {
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
    fn model_uses_only_the_documented_names() {
        assert_eq!(Model::ALL.len(), Model::LABELS.len());
        for (model, name) in Model::ALL.into_iter().zip(Model::LABELS) {
            assert_eq!(model.as_str(), name);
            assert_eq!(model.to_string(), name);
            assert_eq!(name.parse::<Model>().unwrap(), model);
            assert_eq!(
                serde_json::to_string(&model).unwrap(),
                format!("\"{name}\"")
            );
            assert_eq!(
                serde_json::from_str::<Model>(&format!("\"{name}\"")).unwrap(),
                model
            );
        }
        assert!("dso".parse::<Model>().is_err(), "aliases stay rejected");
        assert!(serde_json::from_str::<Model>("\"auto\"").is_err());
    }

    #[test]
    fn dsv_rate_rounds_up_to_an_sflp_rung() {
        assert_eq!(dsv_supported_rate(1), 15);
        assert_eq!(dsv_supported_rate(15), 15);
        assert_eq!(dsv_supported_rate(16), 30);
        assert_eq!(dsv_supported_rate(50), 60);
        assert_eq!(dsv_supported_rate(100), 120);
        assert_eq!(dsv_supported_rate(255), 480);
        assert_eq!(dsv_supported_rate(u16::MAX), 480);
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
        let quaternion = game_rotation([0x3800, 0, 0]).expect("valid quaternion");
        assert!((quaternion[0] - 0.75_f32.sqrt()).abs() < 1e-6);
        assert_eq!(&quaternion[1..], &[0.5, 0.0, 0.0]);
    }

    #[test]
    fn corrupt_game_rotation_is_not_normalised_into_a_measurement() {
        assert_eq!(game_rotation([0x3c00; 3]), None);
        assert_eq!(game_rotation([0x7e00, 0, 0]), None, "NaN");
    }

    #[test]
    fn wrong_tag_and_corrupt_sflp_records_are_not_fresh_measurements() {
        assert_eq!(sflp_game_rotation(false, [0; 6]), None, "wrong tag");
        assert_eq!(
            sflp_game_rotation(true, [0x00, 0x3c, 0x00, 0x3c, 0x00, 0x3c]),
            None,
            "three unit vector components are corrupt"
        );
        assert_eq!(
            sflp_game_rotation(true, [0; 6]),
            Some([1.0, 0.0, 0.0, 0.0]),
            "a valid identity remains fresh"
        );
    }

    #[cfg(target_os = "linux")]
    mod fusion {
        use super::super::linux::DsoFusion;
        use super::super::{GRAVITY, lsm6dso};

        fn level(sequence: u64, gyro: [f32; 3]) -> lsm6dso::RawSample {
            lsm6dso::RawSample {
                sequence,
                gyro,
                accel: [0.0, 0.0, GRAVITY],
            }
        }

        fn rotate(quaternion: [f32; 4], vector: [f32; 3]) -> [f32; 3] {
            let [w, x, y, z] = quaternion;
            [
                (1.0 - 2.0 * (y * y + z * z)) * vector[0]
                    + 2.0 * (x * y - z * w) * vector[1]
                    + 2.0 * (x * z + y * w) * vector[2],
                2.0 * (x * y + z * w) * vector[0]
                    + (1.0 - 2.0 * (x * x + z * z)) * vector[1]
                    + 2.0 * (y * z - x * w) * vector[2],
                2.0 * (x * z - y * w) * vector[0]
                    + 2.0 * (y * z + x * w) * vector[1]
                    + (1.0 - 2.0 * (x * x + y * y)) * vector[2],
            ]
        }

        fn warm(fusion: &mut DsoFusion, first: u64) -> (u64, crate::Sample) {
            for sequence in first..first + 500 {
                if let Some(sample) = fusion.update(level(sequence, [0.0; 3]), 25.0) {
                    return (sequence, sample);
                }
            }
            panic!("Fusion did not converge in its bounded startup interval");
        }

        #[test]
        fn orientation_and_bias_warmup_are_withheld_then_exposed_by_readiness() {
            let mut fusion = DsoFusion::new(100.0).unwrap();
            assert!(fusion.update(level(1, [0.0; 3]), 25.0).is_none());
            assert!(!fusion.ready());

            let (_, sample) = warm(&mut fusion, 2);
            assert!(fusion.ready());
            assert!(sample.quat.iter().all(|value| value.is_finite()));
            assert!((sample.quat[0] - 1.0).abs() < 1e-3, "{:?}", sample.quat);
        }

        #[test]
        fn positive_roll_has_the_sensor_to_world_sign_used_by_the_decoder() {
            let mut fusion = DsoFusion::new(100.0).unwrap();
            let mut sample = None;
            for sequence in 1..=500 {
                sample = fusion.update(
                    lsm6dso::RawSample {
                        sequence,
                        gyro: [0.0; 3],
                        // With NWU axes, world-up expressed by a sensor mounted
                        // at +90 degrees about +X is sensor +Y.
                        accel: [0.0, GRAVITY, 0.0],
                    },
                    25.0,
                );
                if sample.is_some() {
                    break;
                }
            }
            let quaternion = sample.expect("rolled sensor converged").quat;
            let half = std::f32::consts::FRAC_1_SQRT_2;
            assert!((quaternion[0] - half).abs() < 1e-3, "{quaternion:?}");
            assert!((quaternion[1] - half).abs() < 1e-3, "{quaternion:?}");
            assert!(quaternion[2].abs() < 1e-3, "{quaternion:?}");
            assert!(quaternion[3].abs() < 1e-3, "{quaternion:?}");

            let conjugate = [
                quaternion[0],
                -quaternion[1],
                -quaternion[2],
                -quaternion[3],
            ];
            let projected_gravity = rotate(conjugate, [0.0, 0.0, -1.0]);
            for (actual, expected) in projected_gravity.into_iter().zip([0.0, -1.0, 0.0]) {
                assert!(
                    (actual - expected).abs() < 1e-3,
                    "projected gravity {projected_gravity:?} from {quaternion:?}"
                );
            }
        }

        #[test]
        fn restart_withholds_output_until_a_new_complete_warmup() {
            let mut fusion = DsoFusion::new(100.0).unwrap();
            let (last, _) = warm(&mut fusion, 1);
            fusion.restart();
            assert!(!fusion.ready());
            assert!(fusion.update(level(last + 1, [0.0; 3]), 25.0).is_none());
        }

        #[test]
        fn stationary_gyro_bias_is_internal_and_published_rate_stays_raw() {
            let mut fusion = DsoFusion::new(100.0).unwrap();
            let bias = 1.0_f32.to_radians();
            let mut newest = None;
            for sequence in 1..=10_000 {
                newest = fusion.update(level(sequence, [0.0, 0.0, bias]), 25.0);
            }
            let published = newest.expect("filter converged").gyro[2];
            assert!((published - bias).abs() < f32::EPSILON, "{published}");
        }

        #[test]
        fn readiness_waits_for_stationary_bias_then_stays_latched_during_motion() {
            let mut fusion = DsoFusion::new(100.0).unwrap();
            let moving_rate = 4.0_f32.to_radians();

            // AHRS startup can finish while motion continuously prevents the
            // bias estimator's three-second stationary acquisition.
            for sequence in 1..=500 {
                assert!(
                    fusion
                        .update(level(sequence, [0.0, 0.0, moving_rate]), 25.0)
                        .is_none()
                );
            }
            assert!(!fusion.ready());

            let (last, _) = warm(&mut fusion, 501);
            assert!(fusion.ready());

            let published = fusion
                .update(level(last + 1, [0.0, 0.0, moving_rate]), 25.0)
                .expect("motion after calibration stays publishable");
            assert!(fusion.ready());
            assert!((published.gyro[2] - moving_rate).abs() < f32::EPSILON);
        }
    }
}
